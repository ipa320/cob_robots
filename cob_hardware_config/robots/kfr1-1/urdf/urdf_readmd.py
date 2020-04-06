
1.##		添加顏色
  <gazebo reference="fw_steer_link">
    <material>Gazebo/Blue</material>
  </gazebo>


2.##		添加base_footprint與base_link的鏈接關系
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <parent link="base_footprint"/>
    <child link="base_link"/>
  </joint>


3.##		添加transmission傳動關系
  <transmission name="fw_drive_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="fw_drive_joint">
      <hardwareInterface>VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="fw_drive_motor">
      <mechanicalReduction>-79.2380952381</mechanicalReduction>
    </actuator>
  </transmission>


4.##		插件引用
  <gazebo reference="base_link">
    <gravity>true</gravity>
    <self_collide>false</self_collide>
    <sensor name="base_contact_sensor" type="contact">
      <always_on>true</always_on>
      <update_rate>100.0</update_rate>
      <contact>
        <collision>base_link_collision</collision>
        <topic>base_bumper</topic>
      </contact>
      <plugin filename="libgazebo_ros_bumper.so" name="base_gazebo_ros_bumper_controller">
        <frameName>world</frameName>
        <bumperTopicName>base_bumper</bumperTopicName>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo>
    <plugin filename="libgazebo_ros_p3d.so" name="p3d_base_controller">
      <alwaysOn>true</alwaysOn>
      <updateRate>100.0</updateRate>
      <bodyName>base_link</bodyName>
      <topicName>base_pose_ground_truth</topicName>
      <frameName>map</frameName>
      <xyzOffsets>25.7 25.7 0</xyzOffsets>
      <!-- initialize odometry for fake localization-->
      <rpyOffsets>0 0 0</rpyOffsets>
      <gaussianNoise>0.01</gaussianNoise>
    </plugin>
    <canonicalBody>base_footprint</canonicalBody>
  </gazebo>

  <!-- ros_control plugin -->
  <gazebo>
    <plugin filename="libhwi_switch_gazebo_ros_control.so" name="ros_control">
      <robotNamespace>base</robotNamespace>
      <filterJointsParam>joint_names</filterJointsParam>
    </plugin>
  </gazebo>


