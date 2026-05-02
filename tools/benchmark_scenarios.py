#!/usr/bin/env python3
"""Run WorldVM CLI scenarios and report benchmark metadata as JSON."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
EXE = ROOT / "zig-out" / "bin" / "worldvm"
DEFAULT_SCENARIOS = (
    "apple_table",
    "hammer_glass",
    "water_flow",
    "bounce_test",
    "domino_chain",
    "pyramid_collapse",
    "multi_stack",
    "gas_expand",
)
DONE_RE = re.compile(r"Done\. Ticks: (?P<ticks>\d+), Stable: (?P<stable>true|false)")


def run(command: list[str], timeout: int | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )


def parse_run_output(stdout: str) -> tuple[int | None, bool | None]:
    match = DONE_RE.search(stdout)
    if match is None:
        return None, None
    return int(match.group("ticks")), match.group("stable") == "true"


def benchmark_one(scenario: str, ticks: int, run_idx: int, timeout: int | None) -> dict[str, object]:
    command = [str(EXE), "run", "--scenario", scenario, "--ticks", str(ticks)]
    start = time.perf_counter()
    completed = run(command, timeout=timeout)
    elapsed_ms = (time.perf_counter() - start) * 1000.0
    reported_ticks, stable = parse_run_output(completed.stdout)
    return {
        "scenario": scenario,
        "run": run_idx,
        "command": command,
        "returncode": completed.returncode,
        "elapsed_ms": round(elapsed_ms, 3),
        "reported_ticks": reported_ticks,
        "stable": stable,
        "stdout_tail": completed.stdout[-400:],
        "stderr_tail": completed.stderr[-400:],
    }


def scenario_averages(results: list[dict[str, object]]) -> dict[str, float]:
    totals: dict[str, list[float]] = {}
    for result in results:
        if result["returncode"] == 0 and isinstance(result["elapsed_ms"], float):
            totals.setdefault(str(result["scenario"]), []).append(result["elapsed_ms"])
    return {
        scenario: round(sum(values) / len(values), 3)
        for scenario, values in sorted(totals.items())
        if values
    }


def load_baseline(path: str | None) -> dict[str, dict[str, object]]:
    if path is None:
        return {}
    baseline_path = (ROOT / path).resolve()
    data = json.loads(baseline_path.read_text(encoding="utf-8"))
    scenarios = data.get("scenarios")
    if not isinstance(scenarios, dict):
        raise SystemExit("benchmark baseline must contain a scenarios object")
    parsed: dict[str, dict[str, object]] = {}
    for scenario, config in scenarios.items():
        if not isinstance(scenario, str) or not isinstance(config, dict):
            raise SystemExit("benchmark baseline scenario entries must be objects")
        parsed_config: dict[str, object] = {}
        for key in ("max_elapsed_ms", "max_average_ms"):
            value = config.get(key)
            if value is not None:
                if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
                    raise SystemExit(f"baseline {scenario}.{key} must be > 0")
                parsed_config[key] = float(value)
        require_stable = config.get("require_stable")
        if require_stable is not None:
            if not isinstance(require_stable, bool):
                raise SystemExit(f"baseline {scenario}.require_stable must be boolean")
            parsed_config["require_stable"] = require_stable
        parsed[scenario] = parsed_config
    return parsed


def threshold_failures(results: list[dict[str, object]], max_elapsed_ms: float | None, max_average_ms: float | None) -> list[dict[str, object]]:
    failures: list[dict[str, object]] = []
    if max_elapsed_ms is not None:
        for result in results:
            if isinstance(result["elapsed_ms"], float) and result["elapsed_ms"] > max_elapsed_ms:
                failures.append(
                    {
                        "scenario": result["scenario"],
                        "run": result["run"],
                        "elapsed_ms": result["elapsed_ms"],
                        "threshold": "max_elapsed_ms",
                        "limit": max_elapsed_ms,
                    }
                )
    if max_average_ms is not None:
        for scenario, average_ms in scenario_averages(results).items():
            if average_ms > max_average_ms:
                failures.append(
                    {
                        "scenario": scenario,
                        "average_ms": average_ms,
                        "threshold": "max_average_ms",
                        "limit": max_average_ms,
                    }
                )
    return failures


def baseline_failures(results: list[dict[str, object]], baseline: dict[str, dict[str, object]]) -> list[dict[str, object]]:
    failures: list[dict[str, object]] = []
    averages = scenario_averages(results)
    for result in results:
        scenario = str(result["scenario"])
        config = baseline.get(scenario, {})
        max_elapsed_ms = config.get("max_elapsed_ms")
        if isinstance(max_elapsed_ms, float) and isinstance(result["elapsed_ms"], float) and result["elapsed_ms"] > max_elapsed_ms:
            failures.append(
                {
                    "scenario": scenario,
                    "run": result["run"],
                    "elapsed_ms": result["elapsed_ms"],
                    "threshold": "baseline.max_elapsed_ms",
                    "limit": max_elapsed_ms,
                }
            )
        if config.get("require_stable") is True and result["stable"] is not True:
            failures.append(
                {
                    "scenario": scenario,
                    "run": result["run"],
                    "stable": result["stable"],
                    "threshold": "baseline.require_stable",
                    "limit": True,
                }
            )
    for scenario, average_ms in averages.items():
        config = baseline.get(scenario, {})
        max_average_ms = config.get("max_average_ms")
        if isinstance(max_average_ms, float) and average_ms > max_average_ms:
            failures.append(
                {
                    "scenario": scenario,
                    "average_ms": average_ms,
                    "threshold": "baseline.max_average_ms",
                    "limit": max_average_ms,
                }
            )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", action="append", dest="scenarios", help="scenario to run; repeatable")
    parser.add_argument("--ticks", type=int, default=100, help="max ticks per run")
    parser.add_argument("--runs", type=int, default=3, help="runs per scenario")
    parser.add_argument("--timeout", type=int, default=30, help="timeout seconds per run")
    parser.add_argument("--skip-build", action="store_true", help="reuse existing zig-out/bin/worldvm")
    parser.add_argument("--require-stable", action="store_true", help="fail if a scenario does not report stable")
    parser.add_argument("--max-elapsed-ms", type=float, help="fail if any run exceeds this elapsed time")
    parser.add_argument("--max-average-ms", type=float, help="fail if any scenario average exceeds this elapsed time")
    parser.add_argument("--baseline", help="optional benchmark baseline JSON with per-scenario thresholds")
    parser.add_argument("--output", help="optional JSON output path")
    args = parser.parse_args()

    if args.ticks <= 0:
        raise SystemExit("--ticks must be > 0")
    if args.runs <= 0:
        raise SystemExit("--runs must be > 0")
    if args.timeout <= 0:
        raise SystemExit("--timeout must be > 0")
    if args.max_elapsed_ms is not None and args.max_elapsed_ms <= 0:
        raise SystemExit("--max-elapsed-ms must be > 0")
    if args.max_average_ms is not None and args.max_average_ms <= 0:
        raise SystemExit("--max-average-ms must be > 0")

    if not args.skip_build:
        subprocess.run(["zig", "build"], cwd=ROOT, check=True)
    if not EXE.exists():
        raise SystemExit(f"missing executable: {EXE.relative_to(ROOT)}")

    baseline = load_baseline(args.baseline)
    scenarios = tuple(args.scenarios) if args.scenarios else DEFAULT_SCENARIOS
    results: list[dict[str, object]] = []
    for scenario in scenarios:
        for run_idx in range(1, args.runs + 1):
            results.append(benchmark_one(scenario, args.ticks, run_idx, args.timeout))

    failures: list[dict[str, object]] = [
        result
        for result in results
        if result["returncode"] != 0 or result["reported_ticks"] is None or (args.require_stable and result["stable"] is not True)
    ]
    failures.extend(threshold_failures(results, args.max_elapsed_ms, args.max_average_ms))
    failures.extend(baseline_failures(results, baseline))
    payload = {
        "ticks": args.ticks,
        "runs": args.runs,
        "baseline": args.baseline,
        "scenario_count": len(scenarios),
        "scenario_average_ms": scenario_averages(results),
        "thresholds": {
            "max_elapsed_ms": args.max_elapsed_ms,
            "max_average_ms": args.max_average_ms,
            "require_stable": args.require_stable,
        },
        "results": results,
        "failures": failures,
    }
    text = json.dumps(payload, indent=2, sort_keys=True) + "\n"
    if args.output:
        output_path = (ROOT / args.output).resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(text, encoding="utf-8")
    else:
        sys.stdout.write(text)
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
