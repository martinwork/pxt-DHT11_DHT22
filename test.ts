basic.pause(2000)

basic.forever(function () {
    dht11_dht22.queryData(
    DHTtype.DHT11,
    DigitalPin.P0,
    true,
    true,
    true
    )
    if (dht11_dht22.readDataSuccessful()) {
        basic.showString("T: " + ("" + dht11_dht22.readData(dataType.temperature)).substr(0, 2))
        basic.showString("H: " + ("" + dht11_dht22.readData(dataType.humidity)).substr(0, 2))
    } else {
        basic.showString("FAIL")
    }
    dht11_dht22.queryData(
    DHTtype.DHT22,
    DigitalPin.P0,
    true,
    true,
    true
    )
    if (dht11_dht22.readDataSuccessful()) {
        basic.showString("T: " + ("" + dht11_dht22.readData(dataType.temperature)).substr(0, 2))
        basic.showString("H: " + ("" + dht11_dht22.readData(dataType.humidity)).substr(0, 2))
    } else {
        basic.showString("FAIL")
    }
})
