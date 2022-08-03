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
   * Start parsing
   * @return true/false on success/failure.
   */
   bool start();

   /**
   * Stop parsing
   * @return true/false on success/failure.
   */
   bool stop();

   /**
   * Return the status of parsing process
   * @return true/false on running/stopped
   */
   bool running();

   /**
   * Set the update rate
   * @return true/false on success/failure.
   */
   bool set_rate(1:double rate);

   bool set_blur(1:i32 blurtype, 2:i32 bluri);
}
