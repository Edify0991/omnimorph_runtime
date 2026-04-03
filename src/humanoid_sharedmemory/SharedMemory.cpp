#include "SharedMemory.hpp"
#include <unistd.h>
#include <sys/stat.h>
// extern "C" {
// #include "utils.h"
// }

static void ensureFileExists(const std::string& filePath)
{
    size_t pos = filePath.find_last_of('/');
    if (pos != std::string::npos) {
        std::string dir = filePath.substr(0, pos);
        struct stat st{};
        if (::stat(dir.c_str(), &st) != 0) {
            if (mkdir(dir.c_str(), 0777) != 0 && errno != EEXIST) {
                throw std::runtime_error(std::string("Failed to create directory: ") + strerror(errno));
            }
        } else if (!S_ISDIR(st.st_mode)) {
            throw std::runtime_error(dir + " exists but is not a directory");
        }
    }

    struct stat st{};
    if (::stat(filePath.c_str(), &st) != 0) {
        int fd = open(filePath.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd < 0) {
            throw std::runtime_error(std::string("Failed to create file: ") + strerror(errno));
        }
        close(fd);
    }

    if (access(filePath.c_str(), R_OK) != 0) {
        throw std::runtime_error(std::string("File not readable: ") + strerror(errno));
    }
}

/*
 * @brief: Constructor that creates a shared memory segment
 * @param:
 *     filepath - An existing file name or directory name
 *     len      - The size of the shared memory in bytes
 *     key_num  - An integer used to generate the key value
 *     semFlag - Indicates the access control method for the shared memory
 *                0 - mutex lock
 *                1 - read-write lock
 * @return:
 *     None
 *
 * Notes:
 *     As long as filepath and key_num are the same, the generated key value will be identical,
 *     even across different applications.
 */
SharedMemory::SharedMemory(const char *filepath, int len, int key_num, int semFlag, std::string sem_name)
{
    ensureFileExists(filepath);

    this->semFlag = semFlag;
    this->key = ftok(filepath, key_num);
    this->memLength = len;
    if (key < 0) {
        // JC_LOG_ERROR("create key error!");
        throw std::runtime_error("Failed to create key for shared memory");
        return;
    }

    shareMemID = shmget(key, len, IPC_CREAT|0666);
    if (shareMemID < 0) {
        // JC_LOG_ERROR("create share mem error!");
        throw std::runtime_error("Failed to create shared memory segment");
        return;
    }
    // JC_LOG_INFO("Create Share Memory Sucessfully, shareMemID=%d", shareMemID);

    if (semFlag == LOCK_TYPE_MUTEX) {
        sem = sem_open(std::string(sem_name + "_mutex").c_str(), O_CREAT, 0666, 1);
        if (sem == SEM_FAILED) {
            throw std::runtime_error("sem_open failed");
        }
        // JC_LOG_INFO("Mutex Lock Create Sucessfully.");
    } else if(semFlag == LOCK_TYPE_RW) {
        readSem = sem_open(std::string(sem_name + "_read").c_str(), O_CREAT, 0666, 10);
        if (readSem == SEM_FAILED) {
            throw std::runtime_error("sem_open failed for read semaphore");
        }
        writeSem = sem_open(std::string(sem_name + "_write").c_str(), O_CREAT, 0666, 1);
        if (writeSem == SEM_FAILED) {
            throw std::runtime_error("sem_open failed for write semaphore");
        }
        readNumSem = sem_open(std::string(sem_name + "_readnum").c_str(), O_CREAT, 0666, 1);
        if (readNumSem == SEM_FAILED) {
            throw std::runtime_error("sem_open failed for write semaphore");
        }
        // JC_LOG_INFO("Read-Write Lock Create Sucessfully.");
    }

}

SharedMemory::~SharedMemory()
{
    try {
        disconnect();
    } catch (const std::exception&) {
    }

    if (sem && sem != SEM_FAILED) {
        sem_close(sem);
        sem = nullptr;
    }
    if (readSem && readSem != SEM_FAILED) {
        sem_close(readSem);
        readSem = nullptr;
    }
    if (writeSem && writeSem != SEM_FAILED) {
        sem_close(writeSem);
        writeSem = nullptr;
    }
    if (readNumSem && readNumSem != SEM_FAILED) {
        sem_close(readNumSem);
        readNumSem = nullptr;
    }
}

/*
 * @brief  : Connects to the shared memory segment.
 * @param  : None
 * @return : void* - The starting address of the shared memory.
 *
 * Notes:
 *     This function maps the shared memory into the current process's
 *     address space.
 *     'mem' is the address at which the shared memory is connected
 *     in the current process.
 */
void* SharedMemory::connect()
{
    void *mem = shmat(this->shareMemID, NULL, 0);
    if (mem == (void*) -1) {
        // JC_LOG_ERROR("Failed to attach shared memory segment");
        throw std::runtime_error("Failed to attach shared memory segment");
        return NULL;
    }

    this->beginAddress = mem;
    return mem;
}

/*
 * @brief  : Disconnects from the shared memory segment.
 * @param  : None
 * @return : None
 *
 * Notes:
 *     This function detaches the shared memory from the current process's
 *     address space.
 */
void SharedMemory::disconnect()
{
    if (beginAddress == NULL) {
        return;
    }

    if(shmdt(beginAddress) == -1) {
        // JC_LOG_ERROR("Failed to detach shared memory segment.");
        throw std::runtime_error("Failed to detach shared memory segment");
    }
    beginAddress = NULL;
}

/*
 * @brief  : Removes the shared memory segment.
 * @param  : None
 * @return : None
 *
 * Notes:
 *     This function deletes the shared memory segment.
 */
void SharedMemory::remove()
{
    if (shmctl(shareMemID, IPC_RMID, NULL) == -1) {
        // JC_LOG_ERROR("Failed to remove shared memory segment.");
        throw std::runtime_error("Failed to remove shared memory segment");
    }
}

bool SharedMemory::sharedMemValid(void)
{
    return (this->shareMemID >= 0);
}
