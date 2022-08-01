# Copyright: (C) 2020 iCub Fondazione Istituto Italiano di Tecnologia (IIT) 
# All Rights Reserved.
#
# rpc.thrift

/**
* rpc_IDL
*
* IDL Interface
*/
service rpc_IDL
{
   /**
   * Go home with arms and gaze.
   * @return true/false on success/failure.
   */
   bool home();

   /**
   * Grasp the object.
   * @return true/false on success/failure.
   */
   bool grasp();

   /**
   * Go the whole hog.
   * @return true/false on success/failure.
   */
   bool go();
}
