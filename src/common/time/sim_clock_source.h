/*
  © 2022 BLACK SESAME TECHNOLOGIES
 All rights reserved.
 (This product is licensed to: [Hanhai-ADSP Autonomous Driving Solution Platform]
 This computer software is protected by copyright law and international treaties.
 Use, reproduction, or distribution of this software is subject to the license agreement between you and Black Sesame Technologies.
 Unauthorized reproduction or distribution of this program, or any portion of it, may result in severe civil and criminal penalties and will be prosecuted to the maximum extent possible under the law.
*/
#ifndef HANHAI_AUTOPLT_SIM_CLOCK_SOURCE_H
#define HANHAI_AUTOPLT_SIM_CLOCK_SOURCE_H

#include <sys/types.h>
#include <sys/shm.h>
#include <thread>
#include <atomic>
#include <sys/ipc.h>
#include <semaphore.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <mutex>
#define KEY_ID 1234

namespace autoplt
{
    namespace simtime
    {
        struct SimTimeParam
        {
            bool isPaused = false;
            double rate = 1.0;
            uint64_t sim_begin_time = 0;
            uint64_t pause_wall_time = 0;
            uint64_t offset = 0; // 存储偏移量
        };

        class SimClockSource
        {
        public:
            SimClockSource();
            ~SimClockSource();
            int Init(uint64_t begintime, double rate);
            void WriteProcess();
            bool Pause();
            uint64_t &GetSimNow(); // 获取当前的仿真时间

        private:
            static double rate;
            static uint64_t cur_sim_time;
            int shmid = 0;
            std::mutex write_mutex;
            std::atomic<bool> isPausing;
            static uint64_t Now();
            bool CreateShareMemry(uint64_t differ, double rate, uint64_t sim_begin);

        protected:
            void DelShareMem();
        };
    } // namespace simtime

}

#endif
