pub trait ComBusIO {
    fn read_u8() -> u8;
    fn read_u32() -> u32;
    fn read_f32() -> u8;

    fn write_u8();
    fn write_u32();
    fn write_f32();
}
