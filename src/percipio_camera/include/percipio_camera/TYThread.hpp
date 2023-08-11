/*
 * @Description: 
 * @Author: zxy
 * @Date: 2023-08-04 17:56:35
 * @LastEditors: zxy
 * @LastEditTime: 2023-08-08 15:28:50
 */
#ifndef XYZ_TYThread_HPP_
#define XYZ_TYThread_HPP_

class TYThreadImpl;

class TYThread
{
public:
  typedef void* (*Callback_t)(void*);

  TYThread();
  ~TYThread();

  int create(Callback_t cb, void* arg);
  int destroy();

private:
  TYThreadImpl* impl;
};


#endif