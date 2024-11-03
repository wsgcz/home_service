
"use strict";

let GetWaypointByName = require('./GetWaypointByName.js')
let GetChargerByName = require('./GetChargerByName.js')
let SaveWaypoints = require('./SaveWaypoints.js')
let GetNumOfWaypoints = require('./GetNumOfWaypoints.js')
let AddNewWaypoint = require('./AddNewWaypoint.js')
let GetWaypointByIndex = require('./GetWaypointByIndex.js')

module.exports = {
  GetWaypointByName: GetWaypointByName,
  GetChargerByName: GetChargerByName,
  SaveWaypoints: SaveWaypoints,
  GetNumOfWaypoints: GetNumOfWaypoints,
  AddNewWaypoint: AddNewWaypoint,
  GetWaypointByIndex: GetWaypointByIndex,
};
