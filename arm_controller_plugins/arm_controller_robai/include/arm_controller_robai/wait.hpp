/***********************************************************************************
* Copyright: Rose B.V. (2014)
*
* Revision History:
*   Author: Mathijs de Langen
*   Date  : 2014/06/30
*       - File created.
*
* Description:
*   description
* 
***********************************************************************************/
#ifndef WAIT_HPP
#define WAIT_HPP

#include "foundCore/ecMutex.h"

class Wait
{
public:
   bool waitForCompletion()
   {
      EcMutexScopedLock    lock(mutex_);
      completion_received_ = false;

      while ( not completion_received_ )
      {
         ROS_DEBUG("Waiting...");
         condition_.wait(lock);
      }

      return success_;
   }

   void CB_actionCompleted( bool success, void* /*data*/ )
   {
      ROS_DEBUG("Action completed.");
      {
         EcMutexScopedLock lock(mutex_);
         completion_received_    = true;
         success_                = success;
      }

      condition_.notify_all();
   }

private:
   EcMutex     mutex_;
   EcCondition condition_;
   bool        completion_received_;
   bool        success_;
};

#endif //WAIT_HPP