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

   /**
   * Set the min area
   * @return true/false on success/failure.
   */
   bool set_min(1:double minarea);

   /**
   * Set the max area
   * @return true/false on success/failure.
   */
   bool set_max(1:double maxarea);

   bool red();
   bool orange();
   bool green();
   bool yellow();
   bool purple();
   bool blue();

   /**
   * Get the color
   * @return true/false on success/failure.
   */
   string get_color();

   bool set_color(1:i32 newlow1, 2:i32 newlow2, 3:i32 newlow3, 4:i32 newhigh1, 5:i32 newhigh2, 6:i32 newhigh3);

}
