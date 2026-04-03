#pragma once
#include <sys/ipc.h>
#include <sys/shm.h>
#include <stdio.h>
#include <iostream>
#include <cstring>
#include <cerrno>
#include <semaphore.h>
#include <fcntl.h>
#include <chrono>

enum {
    LOCK_TYPE_MUTEX,
    LOCK_TYPE_RW
};

class SharedMemory
{
public:
    SharedMemory(const char *filepath, int len, int key_num, int semFlag, std::string sem_name);
    ~SharedMemory();

    void* connect();
    void disconnect();
    void remove();
    bool sharedMemValid(void);

    template <typename T> void write(T *data, int data_count, int offset=0) {
        auto start = std::chrono::high_resolution_clock::now();

        if (beginAddress == nullptr) {
            throw std::runtime_error("Shared memory is not connected");
        }

        if (offset + (data_count * sizeof(T)) > memLength) {
            // JC_LOG_ERROR("Out of memory range");
            throw std::runtime_error("Out of memory range");
            return;
        }

        if (semFlag==LOCK_TYPE_MUTEX) {
            waitSemaphore(sem);
            memcpy((uint8_t*)beginAddress + offset, data, data_count * sizeof(T));
            sem_post(sem);
        }

        if (semFlag==LOCK_TYPE_RW) {
            waitSemaphore(readSem);
            /**
             * Adding a read lock to protect reader_count++
             * Note:
             * reader_count cannot be shared across multiple processes unless they share
             * the same instance in shared memory.
             **/
            if (readerCount == 0) {
                waitSemaphore(writeSem);
            }

            readerCount++;
            sem_post(readSem);
            memcpy((uint8_t*)beginAddress + offset, data, data_count * sizeof(T));
            waitSemaphore(readSem);
            readerCount--;

            if (readerCount == 0) {
                sem_post(writeSem); // allow user to write
            }
            sem_post(readSem);
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        // JC_LOG("write time: %dus", elapsed);
    };

    template <typename T> void read(T *data, int data_count, int offset=0) {
        auto start = std::chrono::high_resolution_clock::now();

        if (beginAddress == nullptr) {
            throw std::runtime_error("Shared memory is not connected");
        }

        if (offset + data_count * sizeof(T) > memLength) {
            // JC_LOG_ERROR("Out of memory range");
            throw std::runtime_error("Out of memory range");
            return;
        }

        if (semFlag==LOCK_TYPE_MUTEX) {
            waitSemaphore(sem);
            memcpy(data, (uint8_t*)beginAddress + offset, data_count * sizeof(T));
            sem_post(sem);
        }

        if (semFlag==LOCK_TYPE_RW) {
            waitSemaphore(writeSem);
            memcpy(data, (uint8_t*)beginAddress + offset, data_count * sizeof(T));
            sem_post(writeSem);
        }

        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        //JC_LOG("reading time: %dus", elapsed);
    }


private:
    key_t key;
    int shareMemID = -1;
    void *beginAddress = nullptr;
    int memLength;
    int semFlag;
    sem_t *sem = nullptr;
    sem_t *writeSem = nullptr;
    sem_t *readSem = nullptr;
    sem_t *readNumSem = nullptr;
    int readerCount = 0;

    static void waitSemaphore(sem_t *target) {
        if (!target) {
            throw std::runtime_error("invalid semaphore");
        }
        while (sem_wait(target) == -1 && errno == EINTR) {
        }
    }
};
