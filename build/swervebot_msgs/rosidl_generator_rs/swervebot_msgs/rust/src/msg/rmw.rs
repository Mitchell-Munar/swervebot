#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "swervebot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__swervebot_msgs__msg__RobotCommand() -> *const std::ffi::c_void;
}

#[link(name = "swervebot_msgs__rosidl_generator_c")]
extern "C" {
    fn swervebot_msgs__msg__RobotCommand__init(msg: *mut RobotCommand) -> bool;
    fn swervebot_msgs__msg__RobotCommand__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<RobotCommand>, size: usize) -> bool;
    fn swervebot_msgs__msg__RobotCommand__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<RobotCommand>);
    fn swervebot_msgs__msg__RobotCommand__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<RobotCommand>, out_seq: *mut rosidl_runtime_rs::Sequence<RobotCommand>) -> bool;
}

// Corresponds to swervebot_msgs__msg__RobotCommand
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub twist: geometry_msgs::msg::rmw::Twist,


    // This member is not documented.
    #[allow(missing_docs)]
    pub mode: u8,

}

impl RobotCommand {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ACKERMANN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CRAB_WALK: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const ZERO_TURN: u8 = 2;

}


impl Default for RobotCommand {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !swervebot_msgs__msg__RobotCommand__init(&mut msg as *mut _) {
        panic!("Call to swervebot_msgs__msg__RobotCommand__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for RobotCommand {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { swervebot_msgs__msg__RobotCommand__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { swervebot_msgs__msg__RobotCommand__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { swervebot_msgs__msg__RobotCommand__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for RobotCommand {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for RobotCommand where Self: Sized {
  const TYPE_NAME: &'static str = "swervebot_msgs/msg/RobotCommand";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__swervebot_msgs__msg__RobotCommand() }
  }
}


