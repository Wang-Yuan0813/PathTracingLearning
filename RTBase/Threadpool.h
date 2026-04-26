#pragma once
#include <vector>
#include <thread>
#include <mutex>
#include <deque>
#include <functional>
class Threadpool {
private:
	bool _stop;//whether all works have been done
	std::atomic<int> inFlight;
	std::vector<std::thread> workers;
	std::mutex mtx;//we can use lock_guard (prev lecture)
	std::mutex doneMtx;
	std::deque<std::function<void()>> tasks;//queue of works
	std::condition_variable cv;
	std::condition_variable doneCv;
	void workerloop() {//for single thread
		while (true) {
			std::function<void()> task;
			{
				std::unique_lock<std::mutex> lock(mtx);
				cv.wait(lock,[this]{ return _stop || !tasks.empty(); });//if pool is still running but no task, sleep here. 
				if (_stop && tasks.empty()) return;
				task = std::move(tasks.front());
				tasks.pop_front();
			}
			task();
			if(--inFlight == 0){
				std::lock_guard<std::mutex> lock(doneMtx);
				doneCv.notify_all();
			}
		}
	}
public:
	explicit Threadpool(int procNum) {//must have the number of threads
		_stop = false;
		inFlight = 0;
		workers.reserve(procNum);//space allocation
		for (int i = 0; i < procNum; i++) {
			workers.emplace_back([this] {workerloop(); });
		}
	}
	Threadpool(const Threadpool&) = delete;//avoid copy
	Threadpool& operator=(const Threadpool&) = delete;//avoid copy
	void submit(std::function<void()> task) {
		{
			std::lock_guard<std::mutex> lock(mtx);
			if (_stop) throw std::runtime_error("submit on a stopped threadpool");
			//tasks.emplace_back(std::move(task));//no advance, because task has been constructed
			tasks.push_back(std::move(task));
			++inFlight;//tasks increase
		}
		cv.notify_one();
	}
	void waitForTasks() {
		std::unique_lock<std::mutex> lock(doneMtx);
		doneCv.wait(lock, [this] {return inFlight.load() == 0; });
	}
	void shutDown() {//stop threadpool and wait for all threads to finish
		{
			std::lock_guard<std::mutex> lock(mtx);
			if (_stop)	return;
			_stop = true;
		}
		cv.notify_all();
		for (auto& thread : workers) {
			if (thread.joinable()) thread.join();
		}
		workers.clear();
	}
	int size() const {
		return workers.size();
	}
};