
"use strict";

let GetWaypointByIndex = require('./GetWaypointByIndex.js')
let SaveWaypoints = require('./SaveWaypoints.js')
let AddNewWaypoint = require('./AddNewWaypoint.js')
let GetChargerByName = require('./GetChargerByName.js')
let GetWaypointByName = require('./GetWaypointByName.js')
let GetNumOfWaypoints = require('./GetNumOfWaypoints.js')

module.exports = {
  GetWaypointByIndex: GetWaypointByIndex,
  SaveWaypoints: SaveWaypoints,
  AddNewWaypoint: AddNewWaypoint,
  GetChargerByName: GetChargerByName,
  GetWaypointByName: GetWaypointByName,
  GetNumOfWaypoints: GetNumOfWaypoints,
};
