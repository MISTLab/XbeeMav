/* MultithreadingDeque.hpp -- Safe Multithreading Deque class --             */
/* ------------------------------------------------------------------------- */
/* September 20, 2016 -- @Copyright Aymen Soussia. All rights reserved.      */
/*                                  (aymen.soussia@gmail.com)                */


#pragma once

#include<deque>
#include<mutex>


namespace Mist
{


namespace Xbee
{


//*****************************************************************************
template<typename _T>
class MultithreadingDeque
{
public:
	//****************************************************************************
	MultithreadingDeque()
	{
	}


	//****************************************************************************
	~MultithreadingDeque()
	{
	}


	//****************************************************************************
	void Push_Pack(const _T& new_data)
	{
		std::lock_guard<std::mutex> guard(mutex_);
		deque_.push_back(new_data);
	}


	//****************************************************************************
	_T Pop_Front()
	{
		std::lock_guard<std::mutex> guard(mutex_);
		_T value = deque_.front();
		deque_.pop_front();
		return value;
	}


	//****************************************************************************
	unsigned int Get_Size()
	{
		std::lock_guard<std::mutex> guard(mutex_);
		return deque_.size();
	}

private:
	
	std::mutex mutex_;
	std::deque<_T> deque_;
};


}


}
