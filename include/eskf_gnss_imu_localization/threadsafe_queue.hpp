#ifndef _THREADSAFE_QUEUE_
#define _THREADSAFE_QUEUE_

#include <condition_variable>
#include <mutex>
#include <queue>

template <typename T>
class ThreadsafeQueue {
 public:
  ThreadsafeQueue(size_t queueSize) : mQueueSize(queueSize) {}

  ~ThreadsafeQueue() {}

  void push(const T& data) {
    std::unique_lock<std::mutex> lock(mMutex);
    mData.push(data);
    if (mData.size() > mQueueSize) {
      mData.pop();
    }
    mConditionVariable.notify_one();
  }

  void push(T&& data) {
    std::unique_lock<std::mutex> lock(mMutex);
    mData.push(std::move(data));
    if (mData.size() > mQueueSize) {
      mData.pop();
    }
    mConditionVariable.notify_one();
  }

  T pop() {
    std::unique_lock<std::mutex> lock(mMutex);
    mConditionVariable.wait(lock, [this] { return !mData.empty(); });
    T value = std::move(mData.front());
    mData.pop();
    return value;
  }

  bool empty() {
    std::unique_lock<std::mutex> lock(mMutex);
    auto empty = mData.empty();
    mConditionVariable.notify_one();
    return empty;
  }

 private:
  size_t mQueueSize = 5;

  std::queue<T> mData;
  std::mutex mMutex;
  std::condition_variable mConditionVariable;
};

#endif  // _THREADSAFE_QUEUE_