const expectedProductName = "CNLohr lolra ch32v203 goertzel test";
const filter = { vendorId : 0x1209, productId : 0xd035 };
let dev = null;
let loopAbort = false;

const IQHistoryLen = 4096;
var IQHistoryArray = new Uint32Array(IQHistoryLen);
var MPHistoryArray = new Float32Array(IQHistoryLen*2);
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
	dev = null;
}

var playingAudioProcessor = null;
var audioContext = null;

var targetModulation = 0;
var targetGain = 0.0;

function UpdateButtonNames()
{
	document.getElementById( "ToggleAudioButton" ).value = ( targetGain > 0.0 ) ? "Stop Audio" : "Start Audio";
	document.getElementById( "ToggleAudioModulationButton" ).value = ( targetModulation > 0.5 ) ? "FM" : "AM";
}

async function toggleAudioModulation()
{
	if( playingAudioProcessor != null )
	{
		var newVal = 1 - targetModulation;
		targetModulation = newVal;
	}
	UpdateButtonNames();
}

async function toggleAudio()
{
	if( playingAudioProcessor == null )
	{
		var bypass = '\
		class PlayingAudioProcessor extends AudioWorkletProcessor {\
			static get parameterDescriptors() {\
			  return [\
			  	{ name: "gain", defaultValue: 0, },\
			  	{ name: "sampleAdvance", defaultValue: 0.5, },\
				]\
			};\
			constructor() {\
				super();\
				this.rbuffer = new Float32Array(8192); \
				this.rbufferhead = 0|0; \
				this.rbuffertail = 0|0; \
				this.sampleplace = 0.0; \
				this.dcoffset = 0.0; \
				this.totalsampcount = 0|0; \
				\
				this.port.onmessage = (e) => { \
					for( var i = 0|0; i < e.data.length|0; i++ ) \
					{ \
						let n = (this.rbufferhead + (1|0))%(8192|0); \
						if( n == this.rbuffertail ) \
						{ \
							this.rbuffertail = (this.rbuffertail + (1|0))%(8192|0); \
							/*console.log( `Overflow` ); */ \
						} \
						var vv = e.data[i]; \
						this.dcoffset = this.dcoffset * 0.995 + vv * 0.005; \
						this.rbuffer[this.rbufferhead] = vv - this.dcoffset; \
						this.rbufferhead = n; \
					} \
				}; \
			}\
			\
			process(inputs, outputs, parameters) {\
				/*console.log( parameters.gain[0] );*/ \
				/*console.log( this.ingestData );*/ \
				let len = outputs[0][0].length; \
				const sa = Math.fround( parameters.sampleAdvance[0] ); /*float*/ \
				var s = Math.fround( this.sampleplace );      /*float*/ \
				var tail = this.rbuffertail | 0;              /* int*/  \
				var tailnext = this.rbuffertail | 0;          /* int*/  \
				if( tail == this.rbufferhead ) { /*console.log( "Underflow " );*/ return true; }\
				var tsamp = Math.fround( this.rbuffer[tail] ); \
				var nsamp = Math.fround( this.rbuffer[tailnext] ); \
				this.totalsampcount += len|0; \
				for (let b = 0|0; b < len|0; b++) { \
					s += sa; \
					var excess = Math.floor( s ) | 0; \
					if( excess > 0 ) \
					{ \
						s -= excess; \
						tail = ( tail + (excess|0) ) % (8192|0); \
						tailnext = ( tail + (1|0) ) % (8192|0); \
						if( tail == this.rbufferhead ) { /* console.log( "Underflow" ); */ break; } \
						tsamp = Math.fround( this.rbuffer[tail] ); \
						nsamp = Math.fround( this.rbuffer[tailnext] ); \
					} \
					var valv = tsamp * (1.0-s) + nsamp * s; \
					outputs[0][0][b] = valv*parameters.gain[0]; \
				} \
				/*console.log( tail + " " + this.rbuffertail + " " + tsamp + " " + nsamp );*/ \
				this.rbuffertail = tail; \
				this.sampleplace = s; \
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

		audioContext = new AudioContext();

		await audioContext.audioWorklet.addModule(dataURI);

		playingAudioProcessor = new AudioWorkletNode(
			audioContext,
			"playing-audio-processor"
		);
		playingAudioProcessor.connect(audioContext.destination);
		audioContext.resume();

		let gainParam = playingAudioProcessor.parameters.get( "gain" );
		gainParam.setValueAtTime( 0, audioContext.currentTime );
	}

	var newVal = 1.0 - targetGain;
	console.log( "Setting gain to: " + newVal );
	let gainParam = playingAudioProcessor.parameters.get("gain");
	gainParam.setValueAtTime( newVal, audioContext.currentTime);
	targetGain = newVal;
	document.getElementById( "ToggleAudioButton" ).value = ( newVal > 0.5 ) ? "Stop Audio" : "Start Audio";
	UpdateButtonNames();
}

function onLoadWebHidControl()
{
	liveGraph = document.getElementById( "LiveGraph" );
	liveGraph.addEventListener( "mousedown", graphClick );
	liveGraph.addEventListener( "mouseup", graphClick );

	UpdateButtonNames();
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
console.log( "SEND LOOP ERROR" );
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
	sendReport = dev.sendFeatureReport( 0xAC, arraySend ).catch( sendLoopError );
	if( !sendReport ) sendLoopError( "error creating sendFeatureReport" );
}

FMiirphase = 0.0; /* for FM */
FMlastphase = 0.0; /* for FM */
FMphaseout = 0.0; /* for FM */


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
		if( dev && dev !== null && !loopAbort )
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
					(kBsecAvg).toFixed(2) + " kBytes/s<br>" +
					(xActionSecAvg).toFixed(2)  + "transactions/sec<br>";
				document.getElementById( "GeneralData" ).innerHTML =
					"Count: " + goodCount + " / " + badCount + "<br>Inten: " + ((Math.log( lastIntensity * lastIntensity )/Math.log(10)) * 10-120).toFixed(2) + "db (" + lastIntensity + ")";
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

					let grey = darkmode?"rgb( 240 240 240 )":"rgb(16 16 16)";
					let mark = darkmode?"rgb( 255 255 255 )":"rgb(0 0 0)";

					var filledness = lastNumQ * 198 / 120;
					ctx.fillStyle = grey;
					ctx.fillRect( 2, 2 + 198 - filledness, 18, filledness - 2);

					filledness = ( lastTimeUsed * 1.0 / lastTotalTime ) * 198;
					ctx.fillStyle = grey;
					ctx.fillRect( 26, 2 + 198 - filledness, 18, filledness - 2 );

					ctx.fillStyle = mark;

					let mulcoeff = 2500.0 / lastIntensity;
					var lot = 1.2;
					var x = 253;
					for( var i = (IQHistoryHead-1) & (IQHistoryLen-1); i != IQHistoryHead|0; i = (i - 1 + IQHistoryLen) & (IQHistoryLen-1) )
					{
						let power = MPHistoryArray[i*2+0]; //Math.sqrt( real * real + imag * imag ) * mulcoeff;
						let phase = MPHistoryArray[i*2+1]; //Math.atan2( real, imag ) * 0.159155078*0.5;
						ctx.fillRect(x,power*120+10,2,2);
						ctx.fillRect(x,phase*80+110,2,2);

						let v = IQHistoryArray[i];
						let real = (v >> 16);
						let imag = (v & 0xffff);
						if( real > 32767 ) real -= 65536;
						if( imag > 32767 ) imag -= 65536;
						real = real * mulcoeff + 100;
						imag = imag * mulcoeff + 100;
						if( real < 0 ) real = 0; if( real > 255 ) real = 255;
						if( imag < 0 ) imag = 0; if( imag > 255 ) imag = 255;
						x++;
						if( lot > 0 )
						{
							ctx.globalAlpha = lot;
							ctx.fillRect(real+50,imag,2,2);
						}
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
					let sample_divisor  = data[2]&0xffff;
					let demodbuffer = new Float32Array(numq);
					let mulcoeff = 16.0 / lastIntensity;
					for( var i = 0|0; i < numq; i++ )
					{
						let vv = IQHistoryArray[IQHistoryHead] = data[i+3];
						let vi = vv >> 16;
						let vq = vv & 0xffff;
						if( vi >= 32768 ) vi = vi-65535;
						if( vq >= 32768 ) vq = vq-65535;
						let power = Math.sqrt( vi * vi + vq * vq ) * mulcoeff;
						let phase = Math.atan2( vi, vq ) * 0.159155078 + 0.5;
						MPHistoryArray[IQHistoryHead*2+0] = power; //Math.sqrt( real * real + imag * imag ) * mulcoeff;
						MPHistoryArray[IQHistoryHead*2+1] = phase; //Math.atan2( real, imag ) * 0.159155078*0.5;
						if( targetModulation == 0 )
						{ /* AM */
							demodbuffer[i] = power;
						}
						else if( targetModulation == 1 )
						{ /* FM */
							var diffphase = phase - FMlastphase;
							FMlastphase = phase;
							if( diffphase - FMiirphase < 0.0 ) diffphase += 1.0;
							if( diffphase - FMiirphase > 1.0 ) diffphase -= 1.0;
							FMiirphase = FMiirphase * 0.999 + diffphase * 0.001;
							diffphase -= FMiirphase;
							if( diffphase > 0.5 ) diffphase -= 1;
							if( diffphase <-0.5 ) diffphase += 1;
							var po = FMphaseout = FMphaseout * 0.993 + diffphase;
							if( po < 0.0 ) po += 1.0;
							if( po > 1.0 ) po -= 1.0;
							demodbuffer[i] = po * .3; // Turn down FM it's really loud.
						}

						IQHistoryHead = (IQHistoryHead+1)%IQHistoryLen;
					}
					lastIntensity = intensity;
					lastNumQ = numq;
					lastTotalTime = time_total;
					lastTimeUsed = time_used;

					if( audioContext != null && playingAudioProcessor != null )
					{
						// TODO: Use crystalmhz
						let sampleAdvance = (system_rate/sample_divisor) / audioContext.sampleRate;
						let sampleAdvanceParam = playingAudioProcessor.parameters.get("sampleAdvance");
						sampleAdvanceParam.setValueAtTime( sampleAdvance, audioContext.currentTime);
						playingAudioProcessor.port.postMessage( demodbuffer );
					}
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
	console.log( "ABORT\n" );
}

