#include "object_cache_util.hpp"
#include <thread>
#include <unistd.h>
#include <syslog.h>
#include <sched.h>
#include <time.h>

#define LOWEST_PRIORITY (19)

inline bool startDaemon() {
    // are we in the parent process?
    if (fork()) return false;
    // make this process the session leader
    setsid();
    UNUSED auto dirchanged = chdir("/");
    assert (dirchanged == 0);
    umask(0);
    if (fork()) exit(0);
    close(STDIN_FILENO); //redirect to /dev/null?
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
    return true;
}

// white-list the potentially cached files
inline bool isCachedFile(const fs::path & path) {
    system::error_code ec;
    if (BOOST_UNLIKELY(!fs::is_regular_file(path, ec))) return false;
    const auto ext = path.extension();
    return (ext.compare(OBJECT_FILE_EXTENSION) == 0 || ext.compare(KERNEL_FILE_EXTENSION)  == 0);
}

inline void setLowestPriority() {
    UNUSED auto nicesetting = nice(LOWEST_PRIORITY);
    assert (nicesetting != -1);
}

inline void writeTime() {
}

inline int runCacheCleanUp(const fs::path cachePath) noexcept {

    // Iteratively delete any files in "cachePath" that haven't been touched in
    // CACHE_ENTRY_EXPIRY_PERIOD until the process is killed or no more deletable
    // files exist in the directory.

    setLowestPriority();

    if (startDaemon()) {

        openlog(CACHE_JANITOR_FILE_NAME, LOG_PID | LOG_CONS, LOG_DAEMON);
        syslog(LOG_NOTICE | LOG_USER, "starting in %s ...", cachePath.c_str());
        // call nice again even though the priority should have been inherited
        // from the parent process.
        setLowestPriority();

        // TODO: what if the CacheDaysLimit setting changes as this is running?
        try {
            bool createPidFile = true;
            for (;;) {

                // File locks are advisory locks. So if the pid file is deleted
                // and another made while this process is asleep, it would wrongly
                // assume that it still owns the lock. To handle this, when this
                // daemon wakes up it releases and attempts to relock the file.
                // If it fails to do so, it aborts. This does open a window for
                // the ObjectCache to spawn a daemon that is immediately killed
                // but this shouldn't be expected behaviour.

                const FileLock lock(cachePath, createPidFile);
                if (BOOST_UNLIKELY(!lock.locked())) {
                    syslog(LOG_NOTICE | LOG_USER, "cannot lock pid file; stopping...");
                    return -1;
                }
                lock.write_pid();

                sched_yield();
                auto itr = fs::directory_iterator(cachePath);
                const auto end = fs::directory_iterator();
                auto nextCleanUpTime = currentTime() + CACHE_ENTRY_EXPIRY_PERIOD;
                bool isEmpty = true;
                while (BOOST_LIKELY(itr != end)) {
                    const auto e = itr->path();
                    system::error_code ec;
                    itr.increment(ec);
                    if (BOOST_UNLIKELY(!!ec)) {
                        syslog(LOG_EMERG | LOG_USER, "halted due to filesystem corruption.");
                        return -3;
                    }
                    if (BOOST_LIKELY(isCachedFile(e))) {
                        const auto time = fs::last_write_time(e);
                        const auto expiryTime = time + CACHE_ENTRY_EXPIRY_PERIOD;
                        if (BOOST_UNLIKELY(currentTime() < expiryTime)) {
                            nextCleanUpTime = std::min(nextCleanUpTime, expiryTime);
                            isEmpty = false;
                        } else {
                            fs::remove(e);
                            syslog(LOG_INFO | LOG_USER, "removing %s", e.c_str());
                            continue;
                        }
                    }
                    sched_yield();
                }

                // Cache directory is empty; shut the daemon down.
                if (BOOST_UNLIKELY(isEmpty)) {
                    syslog(LOG_NOTICE | LOG_USER, "cache directory is empty; stopping...");
                    return 0;
                }

                // Sleep until the next cleanup round
                char writtenTime[26];
                ctime_r(&nextCleanUpTime, writtenTime);
                writtenTime[24] = '\0';  // Removes the newline that is added
                syslog(LOG_NOTICE | LOG_USER, "sleeping until %s...", writtenTime);
                std::this_thread::sleep_until(system_clock::from_time_t(nextCleanUpTime));
                syslog(LOG_INFO | LOG_USER, "waking...");
                createPidFile = false;
            }
        } catch (std::exception & e) {
            syslog(LOG_ERR | LOG_USER, "halted due to %s", e.what());
            return -2;
        }
    }
    return 0;
}

int main(int argc, const char * const argv[]) {
    if (BOOST_LIKELY(argc == 2)) {
        return runCacheCleanUp(argv[1]);
    } else {
        return -1;
    }
}
