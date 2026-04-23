//! 64-bit Address Encoding for World VM
//! Bits: [world:4][page_x:10][page_y:10][page_z:10][local:10] (per tasks.md)
//! Note: We adjust to fit 1024^3 world with 32^3 pages.
//! Actual mapping used here: [world:4][reserved:15][page_x:5][page_y:5][page_z:5][local_x:5][local_y:5][local_z:5] = 49 bits

const std = @import("std");

pub const WorldAddr = u64;

pub const AddrParts = struct {
    world: u4,
    px: u5,
    py: u5,
    pz: u5,
    lx: u5,
    ly: u5,
    lz: u5,
};

pub fn encode(parts: AddrParts) WorldAddr {
    var addr: u64 = 0;
    addr |= @as(u64, parts.world) << 30;
    addr |= @as(u64, parts.px) << 25;
    addr |= @as(u64, parts.py) << 20;
    addr |= @as(u64, parts.pz) << 15;
    addr |= @as(u64, parts.lx) << 10;
    addr |= @as(u64, parts.ly) << 5;
    addr |= @as(u64, parts.lz);
    return addr;
}

pub fn decode(addr: WorldAddr) AddrParts {
    return .{
        .world = @truncate(addr >> 30),
        .px = @truncate(addr >> 25),
        .py = @truncate(addr >> 20),
        .pz = @truncate(addr >> 15),
        .lx = @truncate(addr >> 10),
        .ly = @truncate(addr >> 5),
        .lz = @truncate(addr),
    };
}

pub fn getPageId(addr: WorldAddr) u32 {
    return @truncate((addr >> 15) & 0x7FFF); // px, py, pz combined (15 bits)
}

pub fn getLocalIdx(addr: WorldAddr) u15 {
    return @truncate(addr & 0x7FFF); // lx, ly, lz combined (15 bits)
}

test "encode/decode address" {
    const parts = AddrParts{
        .world = 1,
        .px = 10, .py = 20, .pz = 30,
        .lx = 5, .ly = 15, .lz = 25,
    };
    const addr = encode(parts);
    const decoded = decode(addr);
    try std.testing.expectEqual(parts, decoded);
}
