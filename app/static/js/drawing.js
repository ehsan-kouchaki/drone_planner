import { globalElements } from './globals.js'
import { changeAddressAltitude } from './addresses.js'
export { drawRoutes, addMarker, drawCircle, drawPolygon, addPoseMarker, deleteMarker, deleteAllMarkers, drawProblemData }
// Adds a marker to the globalElements.mapGoogle
function addMarker (coordinates, index, color = 'red', Opacity = 1) {
  const svgMarker = {
    // Style of the marker
    path: 'M-1.547 12l6.563-6.609-1.406-1.406-5.156 5.203-2.063-2.109-1.406 1.406zM0 0q2.906 0 4.945 2.039t2.039 4.945q0 1.453-0.727 3.328t-1.758 3.516-2.039 3.070-1.711 2.273l-0.75 0.797q-0.281-0.328-0.75-0.867t-1.688-2.156-2.133-3.141-1.664-3.445-0.75-3.375q0-2.906 2.039-4.945t4.945-2.039z',
    fillColor: color,
    fillOpacity: Opacity,
    strokeWeight: 0,
    rotation: 0,
    scale: 1,
    anchor: new google.maps.Point(0, 20)
  }
  if (index < globalElements.markers.length && index !== 0) {
    console.log('Marker already added')
    return 0
  }
  const marker = new google.maps.Marker({
    position: {
      lat: coordinates[0],
      lng: coordinates[1],
      alt: coordinates.length === 3 ? coordinates[3] : 40
    },
    map: globalElements.mapGoogle,
    title: index.toString(),
    //label: index.toString(),
    icon: svgMarker
  })
  globalElements.markers.push(marker) // Add marker
  // The content of each marker
  marker.addListener('mouseover', () => {
    globalElements.infoWindow.close()
    const deleteButton = document.createElement('button')
    deleteButton.classList.add('small-blue-button')
    deleteButton.textContent = 'Delete'
    deleteButton.addEventListener('click', function () {
      deleteMarker(parseInt(marker.getTitle()))
    })
    const textContainer = document.createElement('div')
    textContainer.style.display = 'flex'
    /*const text = document.createElement('p')
    text.style.marginRight = '4px'
    text.textContent =
      'LAT: ' +
      coordinates[0].toFixed(2).toString() +
      ' LNG: ' +
      coordinates[1].toFixed(2).toString() +
      ' ALT: '*/
    const inputAlt = document.createElement('input')
    inputAlt.style.width = '50px'
    inputAlt.style.padding = 0
    inputAlt.type = 'number'
    inputAlt.step = 0.1
    inputAlt.value = coordinates.length === 3 ? coordinates[2] : 40
    // Event listener to change the altitude
    inputAlt.addEventListener('input', function () {
      changeAddressAltitude(index, inputAlt.value)
      if (coordinates.length === 2) {
        coordinates.push(inputAlt.value)
      } else {
        coordinates[2] = inputAlt.value
      }
    })
    textContainer.appendChild(text)
    textContainer.appendChild(inputAlt)
    const deleteButtonContainer = document.createElement('div')
    deleteButtonContainer.appendChild(deleteButton)
    const container = document.createElement('div')
    container.appendChild(textContainer)
    container.appendChild(deleteButtonContainer)
    container.style.display = 'flex'
    container.style.flexDirection = 'column'
    globalElements.infoWindow.setContent(container)
    globalElements.infoWindow.open(marker.getMap(), marker)
  })
}

function drawCircle (center, radius, stroke_color, stroke_weight, fill_color, fill_opacity) {
  const no_fly_region = new google.maps.Circle({
    strokeColor: stroke_color,
    strokeOpacity: 1,
    strokeWeight: stroke_weight,
    fillColor: fill_color,
    fillOpacity: fill_opacity,
    map: globalElements.mapGoogle,
    center: {
       lat: center[0], 
       lng: center[1]
    },
    radius: radius,
  })
  globalElements.markers.push(no_fly_region) // Add no-fly region
}

function drawPolygon(poly_lat, poly_lng, stroke_color, stroke_weight, fill_color, fill_opacity) {
  if (poly_lat.length > 0) {
    const paths = [];
    for (let i = 0; i < poly_lat.length; i++) {
      paths.push({ lat: poly_lat[i], lng: poly_lng[i] });
    }
    // Add the first point again to close the polygon
    paths.push({ lat: poly_lat[0], lng: poly_lng[0] });

    const search_region = new google.maps.Polygon({
      paths: paths,
      strokeColor: stroke_color,
      strokeOpacity: 1,
      strokeWeight: stroke_weight,
      fillColor: fill_color,
      fillOpacity: fill_opacity,
      map: globalElements.mapGoogle
    });

    globalElements.markers.push(search_region); // Add the polygon to the markers array
  }
}

// Deletes a marker from the globalElements.mapGoogle
function deleteMarker (index) {
  const xhr = new XMLHttpRequest()
  xhr.open('POST', '/delete-address', true)
  xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded')
  xhr.onreadystatechange = function () {
    if (xhr.readyState === XMLHttpRequest.DONE) {
      if (xhr.status === 200) {
        const response = JSON.parse(xhr.responseText)
        if (response.success) {
          console.log('Address deleted successfully')
          for (let i = 0; i < globalElements.markers.length; i++) {
            globalElements.markers[i].setMap(null)
          }
          globalElements.markers = []
          drawProblemData(response.problem_data) // To reorder the indexes
        } else {
          // Failed to convert address
          console.error('Failed to convert address:', response.message)
        }
      } else {
        // Failed communication with the server
        console.error('Failed to delete address:', xhr.status)
      }
    }
  }
  // Send the index as the request body
  xhr.send('index=' + encodeURIComponent(index))
}

function deleteAllMarkers () {
  for (let i = 0; i < globalElements.markers.length; i++) {
    globalElements.markers[i].setMap(null)
  }
  globalElements.markers = []
}

// Function to draw the routes
function drawRoutes (routesInfo, problemData) {
  
  const routes = routesInfo.routes
  const startDepots = problemData.start_nodes
  const endDepots = problemData.end_nodes

  Object.entries(routes).forEach(([vNumber, vRoute]) => {
    const coordinates = vRoute.coordinates
    const nodeIds = vRoute.nodes
    const flightPlanCoordinates = []

    // Count how many redundunt depots are in the nodes
    const count_start = startDepots.filter(item => nodeIds.includes(item)).length - 1;
    const count_end = endDepots.filter(item => nodeIds.includes(item)).length - 1;

    for (let i = count_start; i < coordinates.length - count_end; i++) {
      flightPlanCoordinates.push({ lat: coordinates[i][0], lng: coordinates[i][1] })
    }
    globalElements.paths.push(
      new google.maps.Polyline({
        path: flightPlanCoordinates,
        geodesic: true,
        strokeColor: vRoute.color,
        strokeOpacity: 1.0,
        strokeWeight: 3
      })
    )
    globalElements.paths[vNumber].setMap(globalElements.mapGoogle)
  })
}

// Adds a drone pose marker to the map
function addPoseMarker (coordinates, icon) {
  return new google.maps.Marker({
    position: coordinates,
    map: globalElements.mapGoogle,
    icon
  })
}

// Display markers
function drawProblemData (problemData) {  
  for (let i = 0; i < problemData.addresses.length; i++) {
    if (
      problemData.start_nodes.includes(i) &&
      problemData.end_nodes.includes(i)
    ) {
      // Both Departure and Arrival Depot
      addMarker(problemData.addresses[i], i, 'black')
    } else if (problemData.start_nodes.includes(i)) {
      // Departure depot
      addMarker(problemData.addresses[i], i, 'blue')
    } else if (problemData.end_nodes.includes(i)) {
      // Arrival depot
      addMarker(problemData.addresses[i], i, 'green')
    } else {
      // To visit node
      addMarker(problemData.addresses[i], i, 'red')

      //const visited = [25, 36, 16, 17, 13, 14, 29, 30, 11, 20];
      //if (visited.includes(i)) {addMarker(problemData.addresses[i], i, 'black')}
        //else{addMarker(problemData.addresses[i], i, 'red')}      
    }
  }
    //addMarker([37.182520, -6.446869, 40], 1000, 'blue', 0.5)
    //addMarker([37.235113, -6.504663, 40], 1000, 'blue', 0.5)
    //addMarker([37.075779, -6.379924, 40], 1000, 'blue', 0.5)

    //drawCircle([37.182520, -6.446869], 800, "#2f00ff", 1, "#2f00ff", 1)
    //drawCircle([37.235113, -6.504663], 800, "#2f00ff", 1, "#2f00ff", 1)
    //drawCircle([37.075779, -6.379924], 800, "#2f00ff", 1, "#2f00ff", 1)
    //drawCircle([37.245, -6.375], 800, "#2f00ff", 1, "#2f00ff", 1)

  
  //addMarker([37.058861, -6.415050, 40], 1000, 'blue', 0.5)
  //addMarker([37.267586, -6.447347, 40], 1000, 'blue', 0.5)
  //addMarker([37.176203, -6.190320, 40], 1000, 'blue', 0.5)
  //addMarker([37.176203, -6.447347, 40], 1000, 'blue', 0.5)

  // Draw circular internal no-fly zone
  if (problemData.nfz_int_circ_r !== null) {
    for (let i = 0; i < problemData.nfz_int_circ_r.length; i++) {
      drawCircle([problemData.nfz_int_circ_lat[i], problemData.nfz_int_circ_long[i]], problemData.nfz_int_circ_r[i], "#8B0000", 2, "#8B0000", 0.3)
    }
  }  
  // Draw polygonal internal no-fly zone
  if (problemData.nfz_int_poly_lat !== null) {
    for (let i = 0; i < problemData.nfz_int_poly_lat.length; i++) {
      drawPolygon(problemData.nfz_int_poly_lat[i], problemData.nfz_int_poly_long[i], "#8B0000", 2, "#8B0000", 0.3)
    }
  }  
  // Draw circular external no-fly zone
  if (problemData.nfz_ext_circ_r !== null) {
    drawCircle([problemData.nfz_ext_circ_lat, problemData.nfz_ext_circ_long], problemData.nfz_ext_circ_r, "#8B0000", 2, null, 0)
  }
  // Draw polygonal extennal no-fly zone
  if (problemData.nfz_ext_poly_lat !== null) {
    drawPolygon(problemData.nfz_ext_poly_lat, problemData.nfz_ext_poly_long, "#8B0000", 2, null, 0)
  }
  // Draw search areas
  if (problemData.mission_type === 'search'){
    // Circular
    if(problemData.search_circ_r !== null){
      drawCircle([problemData.search_circ_lat, problemData.search_circ_long], problemData.search_circ_r, "#8B0000", 1, "#00FF00", 0.2)
    }
    // Polygonal
    if(problemData.search_poly_lat !== null){
      drawPolygon(problemData.search_poly_lat, problemData.search_poly_long, "#8B0000", 1, "#00FF00", 0.2)
    }
  }
  // Draw artificial no-fly zone
  //if (problemData.art_nfz_lat !== null) {
   // for (let i = 0; i < problemData.art_nfz_lat.length; i++) {
  //    drawCircle([problemData.art_nfz_lat[i], problemData.art_nfz_lon[i]], problemData.art_nfz_r[i], "#007b8bff", 2, "#00648bff", 0.3)
 //   }
 // }   
}
