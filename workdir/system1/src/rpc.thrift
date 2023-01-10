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
   * Grasp the object.
   * @return true/false on success/failure.
   */
   bool grasp(1:double x, 2:double y, 3:double z, 4:double angle, 5:double gx, 6:double gy, 7:double gz );

   bool go();
}
