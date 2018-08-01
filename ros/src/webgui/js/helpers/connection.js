connection_exists = true

function doesConnectionExist() {
    var xhr = new XMLHttpRequest();
    var file = "http://" + window.location.hostname + ':12345';
    var randomNum = Math.round(Math.random() * 10000);
 
    xhr.open('HEAD', file + "?rand=" + randomNum, true);
    xhr.send();
     
    xhr.addEventListener("readystatechange", processRequest, false);
 
    function processRequest(e) {
      if (xhr.readyState == 4) {
        if (xhr.status >= 200 && xhr.status < 304) {
          if(connection_exists == false){
              //alert("reloading connection");
              location.reload();
          }
        } else {
            connection_exists = false;
        }
      }
    }
}

setInterval(doesConnectionExist, 1000);