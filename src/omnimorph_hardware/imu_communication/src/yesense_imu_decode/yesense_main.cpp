#include "yesense_imu_decode/yesense_main.h"

bool YesenseImu::updateImuData() {
  nread = read(fd, buffer, RX_BUF_LEN);
  if (nread > 0) {
    // printf("nread = %d\n", nread);
    memcpy(g_recv_buf + g_recv_buf_idx, buffer, nread);
    g_recv_buf_idx += nread;
  }

  cnt = g_recv_buf_idx;
  pos = 0;
  if (cnt < YIS_OUTPUT_MIN_BYTES) {
    return false;
  }

  while (cnt > (unsigned int)0) {
    int ret = analysis_data(g_recv_buf + pos, cnt, &g_output_info);
    if (analysis_done == ret) /*未查找到帧头*/
    {
      pos++;
      cnt--;
    } else if (data_len_err == ret) {
      break;
    } else if (crc_err == ret || analysis_ok == ret) /*删除已解析完的完整一帧*/
    {
      output_data_header_t *header = (output_data_header_t *)(g_recv_buf + pos);
      unsigned int frame_len = header->len + YIS_OUTPUT_MIN_BYTES;
      cnt -= frame_len;
      pos += frame_len;
      // memcpy(g_recv_buf, g_recv_buf + pos, cnt);

      if (analysis_ok == ret) {
        memcpy(g_recv_buf, g_recv_buf + pos, cnt);
        g_recv_buf_idx = cnt;
        tcflush(fd, TCIFLUSH);
        return true;
      }
    }
  }
  memcpy(g_recv_buf, g_recv_buf + pos, cnt);
  g_recv_buf_idx = cnt;
  tcflush(fd, TCIFLUSH);
  return false;
};

// int main(){
//   YesenseImu yesense_imu;
//   while(1){
//     yesense_imu.updateImuData();
//     sleep(10);
//   }
  
//   return 0;
// }