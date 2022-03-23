#ifndef HW_INFO
#define HW_INFO

/**
 * @brief Get the device specific ID
 * 
 * This function retrieves the device specific ID and stores
 * the result in the handed over uint64_t variable.
 *
 * @param id Pointer to a uint64_t variable to store the retrieved id.
 *
 * @return Zero on success or (negative) error code otherwise.
 */
int get_device_id(uint64_t *id);

enum d_cube_nodes
{
    local_42 = 0x3A7E6410F8902FC1,
    local_56 = 0xDDB146F9D7E47347,
    remote_101 = 0xD2F7DC92EFAB3AA8,
    remote_115 = 0x581CE1C675DC5FE5,
    remote_114 = 0x2622FC58CD12A711,
    remote_100 = 0x9E20DEF234DB1466,
    remote_116 = 0x9167CE78AE9F01BB,
    remote_102 = 0x820549BCA2490F21,
    remote_103 = 0xE7CE51ABA9C6CB4E,
    remote_117 = 0xF3C2F3D466F78322,
    remote_113 = 0xD8296C56ADE2CF70,
    remote_107 = 0x2ADC5B5190E881EA,
    remote_106 = 0x3A42BF0D51268688,
    remote_112 = 0x3683570ABF458FE2,
    remote_104 = 0xAB5EE53E2005044B,
    remote_110 = 0x163EDDD14215BA2,
    remote_111 = 0x53E21AA0F647E886,
    remote_105 = 0x76EAAE9DAD576FC2,
    remote_214 = 0x1C392317085215A3,
    remote_200 = 0x36DE11F16D3954E5,
    remote_201 = 0x4029D597846BB919,
    remote_215 = 0x90652560DC51FD58,
    remote_203 = 0xBBC795BDD1CCDCC9,
    remote_217 = 0xFBC291635A698F21,
    remote_216 = 0x57D710103EA91E2,
    remote_202 = 0x3623C8923CA52EAA,
    remote_206 = 0xED9419461656778F,
    remote_212 = 0x5B543EC123E294E5,
    remote_213 = 0x13BC955D2C3BB15,
    remote_207 = 0xB5C69246B360CD33,
    remote_211 = 0xD4340ECDFA5A43C1,
    remote_205 = 0x66DD93497562D6F6,
    remote_204 = 0xFB10B45496DBE187,
    remote_210 = 0x24E79201193BDF38,
    remote_221 = 0x65B0D0805ADF57E9,
    remote_209 = 0x51A2A390F1D7C0C2,
    remote_208 = 0x408A23C61C6EE92A,
    remote_220 = 0x1AB6BE73D859C802,
    remote_222 = 0xAACD6F506F087011,
    remote_223 = 0xABBC49CC9ADFD79E,
    remote_227 = 0x678283B3ADBCDAF1,
    remote_226 = 0xE725276513B6DD75,
    remote_218 = 0xAEE9703A4206B603,
    remote_224 = 0x2F927BD3667E38A1,
    remote_225 = 0xD8D22474BC9F979D,
    remote_219 = 0x3CFEAD80D9C75710,
    remote_108 = 0x757C7C161253B4EC,
    remote_109 = 0xEF3A5E692D3541E,
    remote_119 = 0xE5112612DBDA65EE,
    remote_118 = 0x58ABAF0E6E1314E2,
};

#endif