#ifndef OBJECT_CACHE_SETTINGS_H
#define OBJECT_CACHE_SETTINGS_H

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <chrono>
#include <stdlib.h>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fstream>

#define OBJECT_FILE_EXTENSION ".o"
#define KERNEL_FILE_EXTENSION ".kernel"
#define CACHE_JANITOR_FILE_NAME "cachejanitord"

#define DAEMON_FILE "cachejanitor.pid"

#define CACHE_ENTRY_MAX_HOURS (7 * 24)
#define SECONDS_PER_HOUR (60 * 60)
#define CACHE_ENTRY_EXPIRY_PERIOD (CACHE_ENTRY_MAX_HOURS * SECONDS_PER_HOUR)

#define UNUSED BOOST_ATTRIBUTE_UNUSED

using namespace boost;
using namespace std::chrono;
namespace fs = boost::filesystem;
using ios = std::ios;

inline time_t currentTime() {
    return system_clock::to_time_t(system_clock::now());
}

struct FileLock {

    FileLock(const fs::path & cachePath, const bool allowFileCreation = true) noexcept
    : mFd(open(fs::path{cachePath / DAEMON_FILE}.c_str(),
               (allowFileCreation ? O_CREAT : O_TRUNC) | O_WRONLY | O_NONBLOCK,
               S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH))
    , mLocked(lock(mFd)) {

    }

    bool locked() const {
        return mLocked;
    }

    void write_pid() const noexcept {
        lseek(mFd, 0, SEEK_SET);
        const auto pid = std::to_string(getpid());
        UNUSED auto written = write(mFd, pid.c_str(), pid.length());
        assert (written == pid.length());
    }

    ~FileLock() noexcept {
        if (locked()) {
            unlock(mFd);
            close(mFd);
        }
    }

private:

    static bool lock(const int fd) noexcept {
        if (fd == -1) return false;
        struct flock lock = {};
        lock.l_type = F_WRLCK;
        #if SEEK_SET != 0
        lock.l_whence = SEEK_SET;
        #endif
        return fcntl(fd, F_SETLK, &lock) == 0;
    }

    static bool unlock(const int fd) noexcept {
        if (fd == -1) return false;
        struct flock lock = {};
        lock.l_type = F_UNLCK;
        #if SEEK_SET != 0
        lock.l_whence = SEEK_SET;
        #endif
        return fcntl(fd, F_SETLK, &lock) == 0;
    }

private:
    const int mFd;
    const bool mLocked;
};

#endif // OBJECT_CACHE_SETTINGS_H
