//! Shared sleep / wake helpers.

const scene32 = @import("scene32.zig");
const entity16 = @import("entity16.zig");

pub const SLEEP_VELOCITY_THRESHOLD: i16 = 5;

pub fn shouldSleep(inst: *scene32.Instance) bool {
    const speed = @abs(inst.vel_x) + @abs(inst.vel_y) + @abs(inst.vel_z);
    const ang_speed = @abs(inst.ang_x) + @abs(inst.ang_y) + @abs(inst.ang_z);
    return speed < SLEEP_VELOCITY_THRESHOLD and ang_speed == 0 and inst.state != .broken;
}

pub fn wakeInstance(inst: *scene32.Instance) void {
    inst.sleep_tick = 0;
    if (inst.state == .resting) {
        inst.state = .idle;
    }
}

pub fn wakeSupportedInstancesAfterBreak(
    instances: []scene32.Instance,
    entities: []const entity16.Entity16,
    broken_idx: u8,
) void {
    if (broken_idx >= instances.len) return;
    const broken_inst = &instances[broken_idx];
    if (broken_inst.entity_id >= entities.len) return;
    const broken_entity = &entities[broken_inst.entity_id];

    var other_idx: usize = 0;
    while (other_idx < instances.len) : (other_idx += 1) {
        if (other_idx == broken_idx) continue;
        const other = &instances[other_idx];
        if (other.state != .resting) continue;
        if (other.entity_id >= entities.len) continue;
        const other_entity = &entities[other.entity_id];

        var supported = false;
        for (0..64) |broken_w_idx| {
            const broken_word = broken_entity.topology[broken_w_idx];
            if (broken_word == 0) continue;
            for (0..64) |broken_b_idx| {
                if ((broken_word & (@as(u64, 1) << @as(u6, @truncate(broken_b_idx)))) == 0) continue;
                const broken_local = (broken_w_idx << 6) | broken_b_idx;
                const broken_x: i32 = @intCast((broken_local >> 4) & 0xF);
                const broken_y: i32 = @intCast(broken_local >> 8);
                const broken_z: i32 = @intCast(broken_local & 0xF);
                const support_x = broken_inst.pos_x + broken_x;
                const support_y = broken_inst.pos_y + broken_y + 1;
                const support_z = broken_inst.pos_z + broken_z;

                for (0..64) |other_w_idx| {
                    const other_word = other_entity.topology[other_w_idx];
                    if (other_word == 0) continue;
                    for (0..64) |other_b_idx| {
                        if ((other_word & (@as(u64, 1) << @as(u6, @truncate(other_b_idx)))) == 0) continue;
                        const other_local = (other_w_idx << 6) | other_b_idx;
                        const other_x: i32 = @intCast((other_local >> 4) & 0xF);
                        const other_y: i32 = @intCast(other_local >> 8);
                        const other_z: i32 = @intCast(other_local & 0xF);
                        if (other.pos_x + other_x == support_x and
                            other.pos_y + other_y == support_y and
                            other.pos_z + other_z == support_z)
                        {
                            supported = true;
                            break;
                        }
                    }
                    if (supported) break;
                }
                if (supported) break;
            }
            if (supported) break;
        }

        if (supported) {
            wakeInstance(other);
        }
    }
}
