
"use strict";

let RestartController = require('./RestartController.js')
let TorqueEnable = require('./TorqueEnable.js')
let SetSpeed = require('./SetSpeed.js')
let SetComplianceMargin = require('./SetComplianceMargin.js')
let StartController = require('./StartController.js')
let SetComplianceSlope = require('./SetComplianceSlope.js')
let StopController = require('./StopController.js')
let SetTorqueLimit = require('./SetTorqueLimit.js')
let SetCompliancePunch = require('./SetCompliancePunch.js')

module.exports = {
  RestartController: RestartController,
  TorqueEnable: TorqueEnable,
  SetSpeed: SetSpeed,
  SetComplianceMargin: SetComplianceMargin,
  StartController: StartController,
  SetComplianceSlope: SetComplianceSlope,
  StopController: StopController,
  SetTorqueLimit: SetTorqueLimit,
  SetCompliancePunch: SetCompliancePunch,
};
