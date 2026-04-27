const LISTEN_HOST = Deno.env.get("LISTEN_HOST") ?? "127.0.0.1";
const LISTEN_PORT = Number(Deno.env.get("LISTEN_PORT") ?? "19091");
const UPSTREAM_BASE_URL = Deno.env.get("UPSTREAM_BASE_URL") ?? "https://anyrouter.top/v1";
const UPSTREAM_API_KEY = Deno.env.get("UPSTREAM_API_KEY") ?? "";
const STRIP_MCP_TOOLS = (Deno.env.get("STRIP_MCP_TOOLS") ?? "1") !== "0";
const LOG_TOOL_SANITIZE = (Deno.env.get("LOG_TOOL_SANITIZE") ?? "1") !== "0";
const ALLOWED_TOOL_TYPES = new Set(
  (Deno.env.get("ALLOWED_TOOL_TYPES") ?? "")
    .split(",")
    .map((s) => s.trim())
    .filter(Boolean),
);

type JsonRecord = Record<string, unknown>;

const HOP_BY_HOP = new Set([
  "connection",
  "keep-alive",
  "proxy-authenticate",
  "proxy-authorization",
  "te",
  "trailers",
  "transfer-encoding",
  "upgrade",
  "content-length",
]);

function copyHeaders(input: Headers): Headers {
  const output = new Headers();
  for (const [key, value] of input.entries()) {
    if (!HOP_BY_HOP.has(key.toLowerCase())) {
      output.set(key, value);
    }
  }
  return output;
}

function isObject(value: unknown): value is JsonRecord {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}

function normalizeFunctionTool(tool: JsonRecord): JsonRecord {
  const out: JsonRecord = { ...tool };

  if (typeof out.strict !== "boolean") {
    out.strict = false;
  }

  if (!isObject(out.parameters)) {
    out.parameters = { type: "object", properties: {} };
  }

  return out;
}

function shouldDropToolByType(toolType: string): boolean {
  if (ALLOWED_TOOL_TYPES.size > 0 && !ALLOWED_TOOL_TYPES.has(toolType)) {
    return true;
  }

  if (!STRIP_MCP_TOOLS) {
    return false;
  }

  if (toolType === "mcp" || toolType === "namespace") {
    return true;
  }

  return false;
}

function sanitizeTools(tools: unknown): { changed: boolean; value: unknown; summary?: string } {
  if (!Array.isArray(tools)) {
    return { changed: false, value: tools };
  }

  let changed = false;
  const kept: unknown[] = [];
  let droppedNamespace = 0;
  let droppedMcp = 0;
  let droppedByAllowList = 0;

  for (const rawTool of tools) {
    if (!isObject(rawTool)) {
      changed = true;
      continue;
    }

    const toolType = String(rawTool.type ?? "");

    if (toolType === "namespace") {
      droppedNamespace += 1;
    }
    if (toolType === "mcp") {
      droppedMcp += 1;
    }

    if (shouldDropToolByType(toolType)) {
      changed = true;
      if (toolType !== "namespace" && toolType !== "mcp") {
        droppedByAllowList += 1;
      }
      continue;
    }

    if (toolType === "function") {
      const fixed = normalizeFunctionTool(rawTool);
      if (fixed !== rawTool) {
        changed = true;
      }
      kept.push(fixed);
      continue;
    }

    kept.push(rawTool);
  }

  const summary = `tools_in=${tools.length},tools_out=${kept.length},dropped_namespace=${droppedNamespace},dropped_mcp=${droppedMcp},dropped_by_allowlist=${droppedByAllowList}`;
  return { changed, value: kept, summary };
}

function sanitizeRequestBody(body: unknown): { body: unknown; changed: boolean; summary?: string } {
  if (!isObject(body)) {
    return { body, changed: false };
  }

  const clone: JsonRecord = { ...body };
  let changed = false;
  let summary: string | undefined;

  const sanitized = sanitizeTools(clone.tools);
  if (sanitized.changed) {
    clone.tools = sanitized.value;
    changed = true;
    summary = sanitized.summary;
  }

  return { body: clone, changed, summary };
}

async function proxy(req: Request): Promise<Response> {
  const inboundUrl = new URL(req.url);
  const upstreamUrl = new URL(inboundUrl.pathname + inboundUrl.search, UPSTREAM_BASE_URL);

  const headers = copyHeaders(req.headers);
  headers.set("host", upstreamUrl.host);

  if (UPSTREAM_API_KEY) {
    headers.set("authorization", `Bearer ${UPSTREAM_API_KEY}`);
  }

  let body: BodyInit | undefined;
  if (req.method !== "GET" && req.method !== "HEAD") {
    const contentType = req.headers.get("content-type") ?? "";
    if (contentType.toLowerCase().includes("application/json")) {
      const raw = await req.text();
      try {
        const parsed = raw ? JSON.parse(raw) : {};
        const sanitized = sanitizeRequestBody(parsed);
        if (LOG_TOOL_SANITIZE && sanitized.summary) {
          console.error(`[proxy] ${sanitized.summary}`);
        }
        body = JSON.stringify(sanitized.body);
      } catch {
        body = raw;
      }
    } else {
      body = await req.arrayBuffer();
    }
  }

  const upstreamResp = await fetch(upstreamUrl, {
    method: req.method,
    headers,
    body,
    redirect: "manual",
  });

  return new Response(upstreamResp.body, {
    status: upstreamResp.status,
    statusText: upstreamResp.statusText,
    headers: copyHeaders(upstreamResp.headers),
  });
}

console.error(
  `[proxy] listening on http://${LISTEN_HOST}:${LISTEN_PORT} -> ${UPSTREAM_BASE_URL} (strip_mcp_tools=${
    STRIP_MCP_TOOLS ? "1" : "0"
  })`,
);

Deno.serve({ hostname: LISTEN_HOST, port: LISTEN_PORT }, async (req) => {
  try {
    return await proxy(req);
  } catch (error) {
    const msg = error instanceof Error ? error.stack ?? error.message : String(error);
    console.error(`[proxy] request failed: ${msg}`);
    return new Response(msg, { status: 502 });
  }
});
