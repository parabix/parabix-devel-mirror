#ifndef WORKQUEUE_H
#define WORKQUEUE_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>

template <typename T>
class WorkQueue {
public:

    WorkQueue(unsigned size)
    : mDone(false) {

    }

    bool pop(T & item) {
        std::unique_lock<std::mutex> lock(mMutex);
        while (empty()) {
            if (BOOST_UNLIKELY(mDone)) {
                item = nullptr;
                return false;
            }
            mCond.wait(lock);
        }
        item = mQueue.front();
        mQueue.pop();
        return true;
    }

    bool empty() const {
        return mQueue.empty();
    }

    bool try_pop(T & item) {
        std::unique_lock<std::mutex> lock(mMutex, std::defer_lock);
        if (lock.try_lock()) {
            item = mQueue.front();
            mQueue.pop();
            return true;
        }
        return false;
    }

    void push(T item) {
        std::unique_lock<std::mutex> lock(mMutex);
        mQueue.push(std::move(item));
        lock.unlock();
        mCond.notify_one();
    }

    void notify_all() {
        mDone = true;
        mCond.notify_all();
    }

private:
    bool                        mDone;
    std::queue<T>               mQueue;
    std::mutex                  mMutex;
    std::condition_variable     mCond;
};




#endif // WORKQUEUE_H
