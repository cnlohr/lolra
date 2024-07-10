const expectedProductName = "CNLohr lolra ch32v203 goertzel test";
const filter = { vendorId : 0x1209, productId : 0xd035 };
let dev = null;
let loopAbort = false;

const IQHistoryLen = 4096;
var IQHistoryArray = new Uint32Array(IQHistoryLen);
var IQHistoryHead = 0|0;
var lastIntensity = 1.0;
var lastNumQ = 0;
var lastTotalTime = 1;
var lastTimeUsed = 1;

var graphIsClicked = false;

function graphClick( e )
{
	if( e.type == "mousedown" ) graphIsClicked = true;
	if( e.type == "mouseup" ) graphIsClicked = false;
	return true;
}

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



async function toggleAudio()
{
var bypass = '\
class PlayingAudioProcessor extends AudioWorkletProcessor {\
	static get parameterDescriptors() {\
	  return [\
	   {\
		  name: "ingestRate",\
		  defaultValue: 25000,\
		  minValue: 1,\
		  maxValue: 1000000\
		},\
		]\
	};\
	constructor() {\
		super();\
		this.port.onmessage = (e) => {\
		      console.log(e.data);\
			  this.ingestData = e.data;\
		};\
		this.ingestData = new ArrayBuffer(0);\
	}\
	\
	process(inputs, outputs, parameters) {\
		/*console.log( parameters.ingestRate[0] );*/ \
		/*console.log( this.ingestData );*/ \
		let len = outputs[0][0].length; \
		for (let b = 0|0; b < len|0; b++) { \
			outputs[0][0][b] = Math.random()*0.01; \
		} \
		return true; \
	} \
} \
\
registerProcessor("playing-audio-processor", PlayingAudioProcessor);';

// The following mechanism does not work on Chrome.
//	const dataURI = URL.createObjectURL( new Blob([bypass], { type: 'text/javascript', } ) );


	// Extremely tricky trick to side-step local file:// CORS issues.
	// https://stackoverflow.com/a/67125196/2926815
	// https://stackoverflow.com/a/72180421/2926815
	let blob = new Blob([bypass], {type: 'application/javascript'});
    let reader = new FileReader();
    await reader.readAsDataURL(blob);
    let dataURI = await new Promise((res) => {
        reader.onloadend = function () {
            res(reader.result);
        }
    });

	var audioContext = new AudioContext();

    await audioContext.audioWorklet.addModule(dataURI);

	PlayingAudioProcessor = new AudioWorkletNode(
		audioContext,
		"playing-audio-processor"
	);
	PlayingAudioProcessor.connect(audioContext.destination);
	audioContext.resume();

	let testParam = PlayingAudioProcessor.parameters.get("ingestRate");
	testParam.setValueAtTime(1000, audioContext.currentTime);

	PlayingAudioProcessor.port.postMessage( new ArrayBuffer(0) );
}

function onLoadWebHidControl()
{
	liveGraph = document.getElementById( "LiveGraph" );
	liveGraph.addEventListener( "mousedown", graphClick );
	liveGraph.addEventListener( "mouseup", graphClick );

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

function updateWebHidDeviceWithParameters( paramlist )
{
	var i = 0|0;
	var arraySend = new Uint8Array(63);
	for( var i = 0|0; i < paramlist.length|0; i++ )
	{
		var vv = paramlist[i] | 0;
		arraySend[i*4+7] = (vv>>0)&0xff;
		arraySend[i*4+8] = (vv>>8)&0xff;
		arraySend[i*4+9] = (vv>>16)&0xff;
		arraySend[i*4+10] = (vv>>24)&0xff;
	}
	arraySend[3] = paramlist.length | 0;

	console.log( arraySend );

	sendReport = dev.sendFeatureReport( 0xAC, arraySend ).catch( sendLoopError );
	if( !sendReport ) sendLoopError( "error creating sendFeatureReport" );
}


async function sendLoop()
{
	const sleep = ms => new Promise(r => setTimeout(r, ms));
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
			receiveReport = dev.receiveFeatureReport( 0xAD ).catch( sendLoopError );
			if( !receiveReport ) sendLoopError( "error creating receiveReport" );

			frameNo++

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
					"Count: " + goodCount + " / " + badCount;
				lastTime = thisTime;
			}
			else if( frameNo % updateStatsPerfPer == 2 )
			{
				const ctx = liveGraph.getContext("2d");

				if( !graphIsClicked )
				{
					let liveGraphContainer = document.getElementById( "LiveGraphContainer" );
					liveGraph.width = liveGraphContainer.clientWidth;
					liveGraph.style.position = 'absolute';

					ctx.clearRect(0, 0, liveGraph.width, liveGraph.height);


					var filledness = lastNumQ * 198 / 120;
					ctx.fillStyle = "rgb( 240 240 240 )";
					ctx.fillRect( 2, 2 + 198 - filledness, 18, filledness - 2);

					filledness = ( lastTimeUsed * 1.0 / lastTotalTime ) * 198;
					ctx.fillStyle = "rgb( 240 240 240 )";
					ctx.fillRect( 26, 2 + 198 - filledness, 18, filledness - 2 );

					ctx.fillStyle = `rgb( 255 255 255 )`;

					let mulcoeff = 10000.0 / lastIntensity;
					var lot = 1.2;
					var x = 253+IQHistoryLen;
					for( var i = (IQHistoryHead+1) & (IQHistoryLen-1); i != IQHistoryHead|0; i = (i + 1) & (IQHistoryLen-1) )
					{
						let v = IQHistoryArray[i];
						let real = (v >> 16);
						let imag = (v & 0xffff);
						if( real > 32767 ) real -= 65536;
						if( imag > 32767 ) imag -= 65536;
						let power = Math.sqrt( real * real + imag * imag ) * mulcoeff;
						let phase = Math.atan2( real, imag ) * 0.159155078*0.5;
						real = real * mulcoeff + 100;
						imag = imag * mulcoeff + 100;
						if( real < 0 ) real = 0; if( real > 255 ) real = 255;
						if( imag < 0 ) imag = 0; if( imag > 255 ) imag = 255;
						ctx.fillRect(x,power+10,2,2);
						ctx.fillRect(x,phase*140+150,2,2);
						x--;
						ctx.globalAlpha = (lot>0)?lot:0;
						ctx.fillRect(real+50,imag,2,2);
						ctx.globalAlpha = 1.0;
						lot -= 0.0015;
					}

					ctx.strokeStyle = "rgb( 128 128 128 )";
					ctx.fillStyle = "rgb( 128 0 0 )";
					ctx.strokeRect( 1, 1, 20, 198 );
					ctx.strokeRect( 25, 1, 20, 198 );
					ctx.strokeRect( 49, 1, 200, 198 );
					ctx.strokeRect( 253, 1, liveGraph.width, 198 );
				}
			}


			if( sendReport )
			{
				await sendReport;
			}
			if( receiveReport )
			{
				let receiveData = await receiveReport;
				if( receiveData && receiveData.buffer )
				{
					let data = new Uint32Array( receiveData.buffer.slice( 0, 508 ) );
					let intensity = data[0]>>8;
					let numq = data[0] & 0xff;
					let time_total = data[1]>>16;
					let time_used  = data[1]&0xffff;
					//console.log( data );
					for( var i = 0|0; i < numq; i++ )
					{
						IQHistoryArray[IQHistoryHead++] = data[i+3];
						if( IQHistoryHead == IQHistoryLen ) IQHistoryHead = 0;
					}
					lastIntensity = intensity;
					lastNumQ = numq;
					lastTotalTime = time_total;
					lastTimeUsed = time_used;
					goodCount++;
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
					//break;
				}
			}
		}
	}
}

