import { globalElements } from './globals.js'
import { addMarker } from './drawing.js'

// Function to get the initial globalElements.mapGoogle
export function getMap (coordinates = null) {

  if (coordinates == null) {
    const coordinatesList = document.getElementById('map-center')
    coordinates = JSON.parse(coordinatesList.getAttribute('coordinates'))
  }

  const mapOptions = {
    center: { lat: coordinates[0][0], lng: coordinates[0][1] },
    zoom: 9.7,
    //zoom: 11,
    mapTypeId: google.maps.MapTypeId.ROADMAP,
    styles: [
      /* General background (light, paper-like) */
      {
        elementType: "geometry",
        stylers: [{ color: "#eef0f1" }]
      },

      /* Remove all labels */
      {
        elementType: "labels",
        stylers: [{ visibility: "off" }]
      },

      /* Roads (very subtle) */
      {
        featureType: "road",
        elementType: "geometry",
        stylers: [{ color: "#ffffff" }]
      },
      {
        featureType: "road",
        elementType: "geometry.stroke",
        stylers: [{ color: "#d0d0d0" }]
      },

      /* Water: oceans, rivers, lakes */
      {
        featureType: "water",
        elementType: "geometry",
        stylers: [{ color: "#b7d9ec" }]
      },

      /* Hide POIs */
      {
        featureType: "poi",
        stylers: [{ visibility: "off" }]
      },

      /* Landscape softening */
      {
        featureType: "landscape",
        elementType: "geometry",
        stylers: [{ color: "#eceeef" }]
      }
    ]
  }

  // Create the map
  globalElements.mapGoogle =
    new google.maps.Map(document.getElementById('map'), mapOptions)

  // ---------- Custom City Labels (OverlayView) ----------
  class CityLabel extends google.maps.OverlayView {
    constructor(position, text) {
      super()
      this.position = position
      this.text = text
    }

    onAdd() {
      this.div = document.createElement("div")
      this.div.style.position = "absolute"
      this.div.style.fontSize = "14px"
      this.div.style.fontWeight = "600"
      this.div.style.color = "#2c2c2c"
      this.div.style.background = "rgba(255,255,255,0.75)"
      this.div.style.padding = "2px 6px"
      this.div.style.borderRadius = "4px"
      this.div.style.whiteSpace = "nowrap"
      this.div.style.pointerEvents = "none"
      this.div.innerText = this.text

      this.getPanes().overlayLayer.appendChild(this.div)
    }

    draw() {
      const projection = this.getProjection()
      const point = projection.fromLatLngToDivPixel(this.position)

      if (point) {
        this.div.style.left = point.x + "px"
        this.div.style.top = (point.y - 10) + "px"
      }
    }

    onRemove() {
      this.div.remove()
    }
  }

  function addCityLabel(name, lat, lng) {
    const label = new CityLabel(
      new google.maps.LatLng(lat, lng),
      name
    )
    label.setMap(globalElements.mapGoogle)
  }

  // Add selected city labels
  addCityLabel("Sevilla", 37.3891, -5.99)
  addCityLabel("Huelva", 37.24, -6.95)
  addCityLabel("Cádiz", 36.748940, -6.402721)
  addCityLabel("Las Cabezas \nde San Juan", 36.96, -6.00)

  // Info window to share between markers
  globalElements.infoWindow = new google.maps.InfoWindow()

  // Listener to add a marker when the map is clicked
  google.maps.event.addListener(globalElements.mapGoogle, 'click', event => {
    const xhr = new XMLHttpRequest()
    xhr.open('POST', '/handle-address', true)
    xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded')

    const address = [event.latLng.lat(), event.latLng.lng()]

    xhr.send(
      'address=' + encodeURIComponent(address) +
      '&depot=' + encodeURIComponent(false)
    )

    xhr.onreadystatechange = function () {
      if (xhr.readyState === XMLHttpRequest.DONE) {
        if (xhr.status === 200) {
          const response = JSON.parse(xhr.responseText)
          if (response.success) {
            addMarker(response.coordinates, response.index)
          } else {
            console.error('Failed to convert address:', response.message)
          }
        } else {
          console.error('Failed to execute frontend action:', xhr.status)
        }
      }
    }
  })
}
