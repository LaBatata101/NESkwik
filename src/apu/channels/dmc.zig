pub const DMC = struct {
    freq: u8,
    direct: u8,
    sample_addr: u8,
    sammple_length: u8,

    const Self = @This();

    pub fn init() Self {
        return .{
            .freq = 0,
            .direct = 0,
            .sample_addr = 0,
            .sample_length = 0,
        };
    }

    // pub fn play
};
