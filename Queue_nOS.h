/*
 * Queue_nOS.h
 * 
 * By Steven de Salas
 * 
 * Defines a templated (generic) class for a Queue_nOS of things.
 * Used for Arduino projects, just #include "Queue_nOS.h" and add this file via the IDE.
 * 
 * Examples:
 * 
 * Queue_nOS<char> Queue_nOS(10); // Max 10 chars in this queue
 * Queue_nOS.push('H');
 * Queue_nOS.push('e');
 * Queue_nOS.count(); // 2
 * Queue_nOS.push('l');
 * Queue_nOS.push('l');
 * Queue_nOS.count(); // 4
 * Serial.print(Queue_nOS.pop()); // H
 * Serial.print(Queue_nOS.pop()); // e
 * Queue_nOS.count(); // 2
 * Queue_nOS.push('o');
 * Queue_nOS.count(); // 3
 * Serial.print(Queue_nOS.pop()); // l
 * Serial.print(Queue_nOS.pop()); // l
 * Serial.print(Queue_nOS.pop()); // o
 * 
 * struct Point { int x; int y; }
 * Queue_nOS<Point> points(5);
 * points.push(Point{2,4});
 * points.push(Point{5,0});
 * points.count(); // 2
 * 
 */

#ifndef Queue_nOS_H
#define Queue_nOS_H

#include <Arduino.h>

template<class T>
class Queue_nOS {
  private:
    int _front, _back, _count;
    T *_data;
    int _maxitems;
  public:
    Queue_nOS(int maxitems = 256) { 
      _front = 0;
      _back = 0;
      _count = 0;
      _maxitems = maxitems;
      _data = new T[maxitems + 1];   
    }
    ~Queue_nOS() {
      delete[] _data;  
    }
    inline int count();
    inline int front();
    inline int back();
    void push(const T &item);
    T peek();
    T pop();
    void clear();
};

template<class T>
inline int Queue_nOS<T>::count() 
{
  return _count;
}

template<class T>
inline int Queue_nOS<T>::front() 
{
  return _front;
}

template<class T>
inline int Queue_nOS<T>::back() 
{
  return _back;
}

template<class T>
void Queue_nOS<T>::push(const T &item)
{
  if(_count < _maxitems) { // Drops out when full
    _data[_back++]=item;
    ++_count;
    // Check wrap around
    if (_back > _maxitems)
      _back -= (_maxitems + 1);
  }
}

template<class T>
T Queue_nOS<T>::pop() {
  if(_count <= 0) return T(); // Returns empty
  else {
    T result = _data[_front];
    _front++;
    --_count;
    // Check wrap around
    if (_front > _maxitems) 
      _front -= (_maxitems + 1);
    return result; 
  }
}

template<class T>
T Queue_nOS<T>::peek() {
  if(_count <= 0) return T(); // Returns empty
  else return _data[_front];
}

template<class T>
void Queue_nOS<T>::clear() 
{
  _front = _back;
  _count = 0;
}

#endif
