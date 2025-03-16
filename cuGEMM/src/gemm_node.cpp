#include "rclcpp/rclcpp.hpp"
#include <vector>

extern void launchGEMM(const float *A, const float *B, float *C, int N, int K, int M);

class VectorAddNode : public rclcpp::Node {
public:
    VectorAddNode() : Node("gemm_node") {
        int N = 2048, K = 1024, M = 2048;
        std::vector<float> A(N * K, 1.0f), B(K * M, 2.0f), C(N * M);

        launchGEMM(A.data(), B.data(), C.data(), N, K, M);

        RCLCPP_INFO(this->get_logger(), "Vector Addition Result:");
        for (int i = 0; i < N; ++i) {
            if (i % 64 == 0)
                RCLCPP_INFO(this->get_logger(), "C[%d] = %f", i * N, C[i * N]);
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorAddNode>());
    rclcpp::shutdown();
    return 0;
}
