let gateway = `ws://${window.location.hostname}/ws`;
let websocket;
window.addEventListener('load', onload);



function onload(event) {
    initWebSocket();
    initButton();
    getCurrentValueLeft();  
    getCurrentValueRight();  
}

function initWebSocket() {
    console.log('open a WebSocket connection. . . ');
    websocket= new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

function onOpen(event) {
    console.log('Connection opened');
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

function onMessage(event) {
    let myObj = JSON.parse(event.data);

    console.log(myObj);
    console.log(myObj.id);
    console.log(myObj.value);

    if (myObj.id == 'LED') 
    {
        if (myObj.value == "1") 
            document.getElementById('state').innerHTML = "ON";
        else
            document.getElementById('state').innerHTML = "OFF";
    }
    if (myObj.id == 'BAT')
    {
        document.getElementById('battery').innerHTML = myObj.value;
        let svgBatteryGreen = document.getElementById('batteryGreen');
        let level = parseFloat(myObj.value) * 57;
        // 57 = max-width( = 300) / maxVoltage (5.2V)
        svgBatteryGreen.setAttribute("width", parseInt(level));
    }

    if (myObj.id == 'SLL')
    {
        console.log("*!*");
        document.getElementById("pwmSSL").value = myObj.value;
        document.getElementById("textSliderValueLeft").innerHTML = myObj.value;
    }

    if (myObj.id == 'SLR')
    {
        document.getElementById("pwmSSR").value = myObj.value;
        document.getElementById("textSliderValueRight").innerHTML = myObj.value;
    }
}

function initButton() {
    console.log("initButton");
    document.getElementById('bON').addEventListener('click', toggleON);
    document.getElementById('bOFF').addEventListener('click', toggleOFF);
}

function toggleON(event) {
    console.log("toggleON");
    websocket.send('bON'); 
}

function toggleOFF(event) {
    console.log("toggleOFF");
    websocket.send('bOFF'); 
}

function getCurrentValueLeft()  
{
    let xhr = new XMLHttpRequest();
    console.log("start Left");

    xhr.onreadystatechange = function() {
        if(this.readyState == 4 && this.status == 200)
        {
            console.log("getCurrentValue Left"); 
            console.log(this.responseText);
            console.log(this.id); 
            document.getElementById("pwmSSL").value = this.responseText;
            document.getElementById("textSliderValueLeft").innerHTML = this.responseText; 
        }
    };  

    xhr.open("GET", "/currentValueLeft", true);  // dadurch wird die Lambda Funktion aktiviert (setup) 
                                                 // dadurch wird der SliderWert geholt (this.responseText)
    console.log("xhr.send 1");
    xhr.send();                                  // und erst durch diesen Aufruf wird die vorbereitete Funktion ausgeführt. 
    console.log("get CurrentValue Left - done!");
                                                 // der Aufruf der Funktion findet aber erst NACH dieser Funktion statt!
}

function getCurrentValueRight()
{
    let xhr = new XMLHttpRequest();
    console.log("start Right");

    xhr.onreadystatechange = function() {
        if(this.readyState == 4 && this.status == 200)
        {
            console.log("getCurrentValue Right"); 
            console.log(this.responseText);
            console.log(this.id); 
            document.getElementById("pwmSSR").value = this.responseText;
            document.getElementById("textSliderValueRight").innerHTML = this.responseText; 
        }
    };  

    xhr.open("GET", "/currentValueRight", true);  
    console.log("xhr.send 1");
    xhr.send();                              
    console.log("get CurrentValue Right - done!");
                                             
}


function updateSliderPWM(element) {
    let sliderValue = element.value; 
    console.log(element.id);         
    if (element.id == "pwmSSL")   
    {
        console.log("update!");
        console.log(element.value);
        console.log(element.id);
        document.getElementById("textSliderValueLeft").innerHTML = element.value; 
        sliderValue = "sLa" + sliderValue;  
    }

    if (element.id == "pwmSSR")   
    {
        console.log("update!");
        console.log(element.value);
        console.log(element.id);
        document.getElementById("textSliderValueRight").innerHTML = element.value; 
        sliderValue = "sLb" + sliderValue;  
    }
    console.log(sliderValue);
    websocket.send(sliderValue);
}
