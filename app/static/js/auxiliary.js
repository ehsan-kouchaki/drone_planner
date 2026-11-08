import { globalElements } from './globals.js'
import { drawRoutes } from './drawing.js'
export { stringNumbersToList, reset, generateViapoints}



function stringNumbersToList (str, numbers2load) {
  const parts = str.split(/[,\s]+/).filter(part => part !== '')
  console.log(numbers2load)
  if (parts.length === 1 && numbers2load === 1) {
    const singleNumber = parseFloat(parts[0])
    return isNaN(singleNumber)
      ? 'Invalid input: Not a valid number'
      : [singleNumber]
  } else if (parts.length > 1) {
    const numbers = parts.map(part => parseFloat(part))
    console.log('NUMBERS ' + numbers)
    const allNumbers = numbers.every(num => !isNaN(num))
    if (allNumbers && numbers.length === numbers2load) {
      return numbers
    } else {
      return false
    }
  } else {
    return false
  }
}

// Function to reset the problem information
function reset (event) {
  event.preventDefault()
  const xhr = new XMLHttpRequest()
  xhr.open('POST', '/reset', true)
  xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded')
  xhr.onreadystatechange = function () {
    if (xhr.readyState === XMLHttpRequest.DONE) {
      if (xhr.status === 200) {
        // Reset parameters
        for (let i = 0; i < globalElements.checkList.length; i++) {
          document.getElementById(globalElements.checkList[i]).value = 0
        }
        // Delete markers
        for (let i = 0; i < globalElements.markers.length; i++) {
          globalElements.markers[i].setMap(null)
        }
        globalElements.markers = [globalElements.markers[0]]
        // Delete routes
        for (let i = 0; i < globalElements.paths.length; i++) {
          globalElements.paths[i].setMap(null)
        }
        globalElements.paths = []
        console.log('Reset done.')
      } else {
        console.error('Failed to reset:', xhr.status)
      }
    }
  }
  xhr.send()
}

// Function to Delet routes and Generate via points
function generateViapoints (event) {
  event.preventDefault()
  const xhr = new XMLHttpRequest()
  xhr.open('POST', '/generate-via-points', true)
  xhr.setRequestHeader('Content-Type', 'application/x-www-form-urlencoded')
  // Check for any invalid value
  let returnedParam
  for (let i = 0; i < globalElements.checkList.length; i++) {
    returnedParam = noWarning(globalElements.checkList[i], globalElements.checkList[i] + '-warning')
    if (!returnedParam) {
      return false
    }
  }  
  xhr.onreadystatechange = function () {
    if (xhr.readyState === XMLHttpRequest.DONE) {
      if (xhr.status === 200) {
        // Delete routes
        for (let i = 0; i < globalElements.paths.length; i++) {
          globalElements.paths[i].setMap(null)
        }
        globalElements.paths = []
        console.log('Reset done.')
        /*const response = JSON.parse(xhr.responseText)
        if (response.success) {        
          // Draw the routes
          drawRoutes(response.viapoints)
        } else {
          console.error('Route generation failed:', response.message)
        }*/        
      } else {
        console.error('Failed to reset:', xhr.status)
      }
    }
  }
  // Send the parameters
  let url = ''
  for (let i = 0; i < globalElements.checkList.length - 1; i++) {
    url +=
      globalElements.checkList[i] +
      '=' +
      encodeURIComponent(document.getElementById(globalElements.checkList[i]).value) +
      '&'
  }
  //url +=
  //  'travel_mode' +
  //  '=' +
  //  encodeURIComponent(document.getElementById('travel_mode').value) +
  //  '&'
  url +=
    globalElements.checkList[globalElements.checkList.length - 1] +
    '=' +
    encodeURIComponent(
      document.getElementById(globalElements.checkList[globalElements.checkList.length - 1]).value
    )
  xhr.send(url)
}

