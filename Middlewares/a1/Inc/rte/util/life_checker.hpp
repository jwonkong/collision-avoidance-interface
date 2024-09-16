#ifndef LIFE_CHECKER_HPP_
#define LIFE_CHECKER_HPP_

#include <string>
#include <unordered_map>

namespace util {
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

template<typename counter_type>
class LifeChecker {
	typedef enum { kLifeOK = 0 , kLifeWarning, kLifeError} LifeStatus;

public:
	LifeChecker(){};
	~LifeChecker(){};

public:
  void setTick(const uint16_t tick_ms){
	  tick_ = tick_ms;
  }
  void registerList(const std::string& name, const uint16_t& period_ms, const uint16_t& warning_criteria = 3, const uint16_t& error_criteria = 6){
	  prev_counter_[name] = 0;
	  check_counter_[name] = 0;
	  ok_counter_[name] = 0;
	  status_[name] = LifeStatus::kLifeError;
	  warning_criteria_[name] = (counter_type)(warning_criteria*(period_ms/tick_));
	  error_criteria_[name] = (counter_type)(error_criteria*(period_ms/tick_));
  };
  bool check(const std::string& name, const counter_type& counter){
	  LifeStatus status = status_[name];
	  // check life count of command message
	  if (prev_counter_[name] == counter) {
		  if (check_counter_[name] > error_criteria_[name]){
			  status = LifeStatus::kLifeError;
			  ok_counter_[name] = 0;
		  }
		  else if (check_counter_[name] > warning_criteria_[name]){
			  status = LifeStatus::kLifeWarning;
			  check_counter_[name] = check_counter_[name] + 1;
			  ok_counter_[name] = 0;
		  }
		  else{
			  check_counter_[name] = check_counter_[name] + 1;
		  }
	  } else {
		  check_counter_[name] = 0;
		  prev_counter_[name] = counter;
		  if(ok_counter_[name] < 20) ok_counter_[name] = ok_counter_[name] + 1;
		  if(ok_counter_[name] > 10) status = LifeStatus::kLifeOK;
	  }
	  status_[name] = status;
	  return status == LifeStatus::kLifeOK;
  };
  LifeStatus getStatus(const std::string& name){
  	  return status_[name];
  }
  LifeStatus getTotalStatus(void){
	  LifeStatus status = LifeStatus::kLifeOK;
	  for(auto& item: status_){
		  if((status == LifeStatus::kLifeOK)
				  && (item == LifeStatus::kLifeWarning)){
			  status = LifeStatus::kLifeWarning;
		  }
		  if(item == LifeStatus::kLifeError){
			  status = LifeStatus::kLifeError;
			  break;
		  }
	  }
	  return status;
  }

private:
  uint16_t tick_;
  std::unordered_map<std::string, counter_type> prev_counter_;
  std::unordered_map<std::string, counter_type> check_counter_;
  std::unordered_map<std::string, counter_type> ok_counter_;
  std::unordered_map<std::string, counter_type> warning_criteria_;
  std::unordered_map<std::string, counter_type> error_criteria_;
  std::unordered_map<std::string, LifeStatus> status_;
};
}  // namespace util

#endif
