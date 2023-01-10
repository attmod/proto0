# Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
#                    Istituto Italiano di Tecnologia, Genova
# Licence: GPLv2 (please see LICENSE file)

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
