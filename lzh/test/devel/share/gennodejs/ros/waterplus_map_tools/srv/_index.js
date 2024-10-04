
"use strict";

let GetWaypointByIndex = require('./GetWaypointByIndex.js')
let SaveWaypoints = require('./SaveWaypoints.js')
let GetWaypointByName = require('./GetWaypointByName.js')
let GetNumOfWaypoints = require('./GetNumOfWaypoints.js')
let GetChargerByName = require('./GetChargerByName.js')
let AddNewWaypoint = require('./AddNewWaypoint.js')

module.exports = {
  GetWaypointByIndex: GetWaypointByIndex,
  SaveWaypoints: SaveWaypoints,
  GetWaypointByName: GetWaypointByName,
  GetNumOfWaypoints: GetNumOfWaypoints,
  GetChargerByName: GetChargerByName,
  AddNewWaypoint: AddNewWaypoint,
};
