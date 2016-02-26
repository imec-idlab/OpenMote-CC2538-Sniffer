/**
 * @file       Callback.h
 * @author     Pere Tuset-Peiro (peretuset@openmote.com)
 * @version    v0.1
 * @date       May, 2015
 * @brief
 *
 * @copyright  Copyright 2015, OpenMote Technologies, S.L.
 *             This file is licensed under the GNU General Public License v2.
 */

#ifndef CALLBACK_H_
#define CALLBACK_H_

/*****************************************************************************/

typedef void(*callback_t)(void);

/*****************************************************************************/

class Callback
{
public:
    virtual void execute(void) = 0;
private:
};

/*****************************************************************************/

template<typename T>
class GenericCallback : public Callback
{
public:
    GenericCallback(T* object_ = nullptr, \
                   void(T:: *method_)(void) = nullptr):
                   object(object_), method(method_){}
    void execute(void) {(object->*method)();}
private:
    T* object;
    void(T:: *method)(void);
};

/*****************************************************************************/

class PlainCallback : public Callback
{
public:
    PlainCallback(callback_t callback_){callback = callback_;}
    void execute(void){callback();}
private:
    callback_t callback;
};

/*****************************************************************************/

#endif /* CALLBACK_H_ */
