const expectedProductName = "CNLohr lolra ch32v203 goertzel test";
const filter = { vendorId : 0x1209, productId : 0xd035 };
let dev = null;
let loopAbort = false;

function setStatus( msg )
{
	document.getElementById( "STATUS" ).innerHTML = msg;
	console.log( msg );
}

function setStatusError( msg )
{
	setStatus( "<FONT COLOR=RED>" + msg + "</FONT>" );
	console.log( msg );
	console.trace();
}

function tryConnect()
{
	if( !navigator.hid )
	{
		return;
	}
	
	if( !dev )
	{
		navigator.hid.getDevices().then( (devices) =>
		{
			if( devices.length == 0 )
				setStatusError( "No devices found. Open a device." );
			else
				devices.forEach( tryOpen );
		});
	}
}

async function closeDeviceTool()
{
	loopAbort = false;
	setStatusError( "Disconnected" );
}

function onLoadWebHidControl()
{
	setTimeout( sendLoop, 1 );

	if( !navigator.hid )
	{
		setStatusError( "Browser does not support HID." );
		document.getElementById( "connectButton" ).hidden = true;
	}
	else
	{
		navigator.hid.addEventListener("disconnect", (event) => { if( event.device.productName == expectedProductName ) closeDeviceTool(); } );
	}

    setTimeout( () => { elapsedOK = true; }, 3000 );

}

function reqConnect()
{
	loopAbort = true;
	const initialization = navigator.hid.requestDevice( { filters: [ filter ] } );
	initialization.then( gotUSBDevice );
	initialization.catch( setStatusError );
}

function gotUSBDevice(result)
{
	if( result.length < 1 )
	{
		setStatusError( "Error: No devices found" );
		return;
	}

	if( result[0].productName != expectedProductName )
	{
		setStatusError( "Error: Wrong device name.  Got " + result[0].productName + " Expected " + expectedProductName );
		return;
	}

	const thisDev = result[0];
	
	tryOpen( thisDev );
}

function tryOpen( thisDev )
{
	thisDev.open().then( ( result ) => {
		if( result === undefined )
		{
			if( dev ) dev.close();
			loopAbort = false;
			dev = thisDev;
			setStatus( "Connected." );
		}
		else
		{
			setStatusError( "Error: Could not open; " + result );
		}
	} ).catch( (e) => setStatusError( "Error: Could not open; " + e ) );
}

let sendReport = null;
let receiveReport = null;

async function sendLoopError( e )
{
	sendReport = null;
	receiveReport = null;
	if( dev ) await dev.close();
	dev = null;
	setStatusError( e );
}

async function sendLoop()
{
	const sleep = ms => new Promise(r => setTimeout(r, ms));
	var arraySend = new Uint8Array(255);
	var frameNo = 0|0;
	var lastTime = performance.now();
	let goodCount = 0;
	let badCount = 0;
	let kBsecAvg = 0;
	let xActionSecAvg = 0;
	while( true )
	{
		if( dev && !loopAbort )
		{
			var i = 0|0;
			for( var i = 0|0; i < 255|0; i++ )
				arraySend[i] = (Math.random()*256)|0;

			sendReport = dev.sendFeatureReport( 0xAA, arraySend ).catch( sendLoopError );
			if( !sendReport ) sendLoopError( "error creating sendFeatureReport" );

			receiveReport = dev.receiveFeatureReport( 0xAA ).catch( sendLoopError );
			if( !receiveReport ) sendLoopError( "error creating receiveReport" );

			frameNo++;

			const updateStatsPerfPer = 4;
			if( frameNo % updateStatsPerfPer == 0 )
			{
				let thisTime = performance.now();
				let deltaTime = thisTime - lastTime;
				let kBsec = (255*1000/1024*updateStatsPerfPer)/(deltaTime);
				let xActionSec = (2*updateStatsPerfPer*1000)/(deltaTime);
				kBsecAvg = kBsecAvg * 0.9 + kBsec * 0.1;
				xActionSecAvg = xActionSecAvg * 0.9 + xActionSec * 0.1;

				document.getElementById( "StatusPerf" ).innerHTML = 
					(kBsecAvg).toFixed(2) + " kBytes/s (Split between send and receive)<br>" +
					(xActionSecAvg).toFixed(2)  + "transactions/sec<br>" +
					"Good Count: " + goodCount + "<BR>Bad Count: " + badCount;
				lastTime = thisTime;
			}

			if( sendReport )
			{
				await sendReport;
			}
			if( receiveReport )
			{
				// Validate Data.
				let receiveData = await receiveReport;
				if( receiveData && receiveData.buffer )
				{
					let data = new Uint8Array( receiveData.buffer );

					// Tricky: Data goes:
					//  reportID -> Payload...
					let failed = false;
					for( var i = 0|0; i < 254|0; i++ )
					{
						if( data[i+1] != arraySend[i] )
						{
							console.log( "Disagreement at index " + i );
							console.log( data );
							console.log( arraySend );
							badCount++;
							failed = true;
							break;
						}
					}
					if( !failed ) goodCount++;
				}
				else
				{
					badCount++;
				}
			}
		}
		else if( loopAbort )
		{
			if( dev )
			{
				console.log( "Loop Aborting, Dev Closing" );
				await dev.close();
				console.log( "Loop Aborting, Dev Closed." );
				dev = null;
			}
			await sleep(100);
		}
		else
		{
			// Try opening dev.
			console.log( "Attempting reconnect." );
			tryConnect();
			goodCount = 0;
			badCount = 0;
			let i = 0|0;
			for( i = 0; i < 10; i++ )
			{
				await sleep(100);
				if( dev )
				{
					break;
				}
			}
		}
	}
}

