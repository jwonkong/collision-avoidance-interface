#ifndef MUTEX_HPP__
#define MUTEX_HPP__

namespace util {

class mutex {
public:
	mutex() {};
	~mutex() {};

	/**
	 * lock methods
	 */
	inline void lock(){
		while(semaphore_){
			if(!semaphore_) break;
		};
		while(semaphore_read_){
			if(!semaphore_read_) break;
		};
		semaphore_ = true;
		semaphore_read_ = true;
	};
	inline void shared_lock() {
		while(semaphore_){
			if(!semaphore_) break;
		};
		semaphore_read_ = true;
	};
	inline bool try_lock(){
		if(!semaphore_ && !semaphore_read_){
			semaphore_ = true;
			semaphore_read_ = true;
			return true;
		}
		else{
			return false;
		}
	}
	/**
	 * unlock
	 */
	inline void unlock() {
		semaphore_ = false;
		semaphore_read_ = false;
	};

private:
	bool semaphore_;
	bool semaphore_read_;
};
}

#endif
