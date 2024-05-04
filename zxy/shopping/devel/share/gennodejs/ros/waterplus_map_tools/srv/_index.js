
"use strict";

let GetChargerByName = require('./GetChargerByName.js')
let SaveWaypoints = require('./SaveWaypoints.js')
let GetNumOfWaypoints = require('./GetNumOfWaypoints.js')
let GetWaypointByIndex = require('./GetWaypointByIndex.js')
let AddNewWaypoint = require('./AddNewWaypoint.js')
let GetWaypointByName = require('./GetWaypointByName.js')

module.exports = {
  GetChargerByName: GetChargerByName,
  SaveWaypoints: SaveWaypoints,
  GetNumOfWaypoints: GetNumOfWaypoints,
  GetWaypointByIndex: GetWaypointByIndex,
  AddNewWaypoint: AddNewWaypoint,
  GetWaypointByName: GetWaypointByName,
};
