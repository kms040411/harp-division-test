#include <iostream>
#include <string>
#include <unistd.h>
#include <cstdlib>
#include <ctime>
#include <random>
#include <queue>
#include <vector>

#include "opae_svc_wrapper.h"
#include "csr_mgr.h"
#include "afu_json_info.h"

using namespace std;
using namespace opae::fpga::types;
using namespace opae::fpga::bbb::mpf::types;

#define TEST_ITER_NUM 10
typedef int32_t fpga_int;

const int FIFO_DEPTH = 6;

inline void assert_test(int test_num, fpga_int answer, fpga_int result){
    if(answer != result) {
        cout << "Test " << test_num << " failed: FPGA returned (" << result << "), not (" << answer << ")" << endl;
        exit(-1);
    }
}

inline void init_buffer(volatile fpga_int *buffer, int size) {
    for(int i=0; i<size; i++){
        buffer[i] = 0;
    }
}

int main(int argc, char **argv){
    OPAE_SVC_WRAPPER fpga(AFU_ACCEL_UUID);
    if(!fpga.isOk()) return -1;
    CSR_MGR csrs(fpga);

    // Random Integer Generator
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<fpga_int> value_gen(0, 32767);

    // Allocate shared memory space
    auto input_buf_handle = fpga.allocBuffer(getpagesize());
    auto input_buf = reinterpret_cast<volatile fpga_int*>(input_buf_handle->c_type());
    if(!input_buf) return -1;

    auto output_buf_handle = fpga.allocBuffer(getpagesize());
    auto output_buf = reinterpret_cast<volatile fpga_int*>(output_buf_handle->c_type());
    if(!output_buf) return -1;

    /********************************************
     *  CSR Mapping                             *
     *  0: Start Signal                         *   -> [x]: start signal
     *  1: Input Buffer Address                 *   -> [0]: num 1   (32bit)
     *                                          *      [1]: num 2   (32bit)
     *  2: Output Buffer Address                *   -> [0]: end of operation flag (1bit + 31bit zeros)
     *                                          *      [1]: src_data_w      (32bit)
     *  3: Reset Signal                         *   -> [x]: reset signal
     ********************************************/
    // Tell FPGA shared memory addresses
    csrs.writeCSR(1, intptr_t(input_buf));
    csrs.writeCSR(2, intptr_t(output_buf));

    for(int i=0; i<TEST_ITER_NUM; i++){
        fpga_int a = value_gen(gen);
        fpga_int b = value_gen(gen);
        init_buffer(input_buf, 2);
        init_buffer(output_buf, 2);
        
        input_buf[0] = a;
        input_buf[1] = b;
        printf("%d * %d?\n", a, b);

        csrs.writeCSR(0, 1);
        while(output_buf[0] == 0) usleep(1);

        assert_test(1, a * b, output_buf[1]);
        printf("%d * %d == %d\n", a, b, output_buf[1]);

        csrs.writeCSR(3, 1);
    }


    return 0;
}