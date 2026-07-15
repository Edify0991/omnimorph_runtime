#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "rl_master/dds_protocol.h"

int main()
{
    rl_master::RobotCommandData source;
    source.protocol_version = rl_master::kProtocolVersionDynamicJointsV2;
    source.active_joint_count = 4;
    source.open_rl = rl_master::kOpenRlTestR1Stream;
    source.joint_target_q = {0.1f, 0.2f, 0.3f, 0.4f};
    source.joint_target_dq = {0.0f, 0.0f, 0.0f, 0.0f};
    source.joint_target_tau = {0.0f, 3.0f, 4.0f, 0.0f};
    source.joint_cst_mask = {0U, 1U, 1U, 0U};

    const auto encoded = rl_master::dds::encodeRuntimeCommand(source, 17U, 123.5);
    rl_master::RobotCommandData decoded;
    uint32_t sequence = 0;
    double stamp = 0.0;
    if (!rl_master::dds::decodeRuntimeCommand(encoded, &decoded, &sequence, &stamp) ||
        decoded.open_rl != rl_master::kOpenRlTestR1Stream ||
        decoded.joint_cst_mask != source.joint_cst_mask || sequence != 17U ||
        std::abs(stamp - 123.5) > 1e-6)
    {
        std::cerr << "test-R1 optional CST selection V2 round-trip failed\n";
        return 1;
    }

    source.joint_cst_mask.clear();
    const auto legacy_shape = rl_master::dds::encodeRuntimeCommand(source, 18U, 124.0);
    if (!rl_master::dds::decodeRuntimeCommand(legacy_shape, &decoded, &sequence, &stamp) ||
        !decoded.joint_cst_mask.empty())
    {
        std::cerr << "legacy test-R1 payload compatibility failed\n";
        return 1;
    }

    source.joint_cst_mask = {1U};
    const auto malformed_selection = rl_master::dds::encodeRuntimeCommand(source, 19U, 124.5);
    if (!rl_master::dds::decodeRuntimeCommand(malformed_selection, &decoded, &sequence, &stamp) ||
        decoded.joint_cst_mask != std::vector<uint8_t>({0U, 0U, 0U, 0U}))
    {
        std::cerr << "malformed CST selection did not fail safe to all-CSP\n";
        return 1;
    }
    return 0;
}
