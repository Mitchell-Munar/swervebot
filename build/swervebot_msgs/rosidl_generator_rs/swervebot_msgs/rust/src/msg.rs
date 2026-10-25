#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to swervebot_msgs__msg__RobotCommand

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotCommand {

    // This member is not documented.
    #[allow(missing_docs)]
    pub twist: geometry_msgs::msg::Twist,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RobotCommand::default())
  }
}

impl rosidl_runtime_rs::Message for RobotCommand {
  type RmwMsg = super::msg::rmw::RobotCommand;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        twist: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Owned(msg.twist)).into_owned(),
        mode: msg.mode,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        twist: geometry_msgs::msg::Twist::into_rmw_message(std::borrow::Cow::Borrowed(&msg.twist)).into_owned(),
      mode: msg.mode,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      twist: geometry_msgs::msg::Twist::from_rmw_message(msg.twist),
      mode: msg.mode,
    }
  }
}


