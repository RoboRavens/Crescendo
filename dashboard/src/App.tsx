import React from 'react';
import './App.css';
import { ThemeProvider, createTheme } from '@mui/material/styles';
import CssBaseline from '@mui/material/CssBaseline';
import { NetworkTables, NetworkTablesTopic, NetworkTablesTypeInfos } from 'ntcore-ts-client';

import ListItemButton from '@mui/material/ListItemButton';
import ListItemIcon from '@mui/material/ListItemIcon';
import ListItemText from '@mui/material/ListItemText';
import Divider from '@mui/material/Divider';
import ListSubheader from '@mui/material/ListSubheader';

import SmartToyOutlinedIcon from '@mui/icons-material/SmartToyOutlined';
import SportsEsportsOutlinedIcon from '@mui/icons-material/SportsEsportsOutlined';
import CarRepairOutlinedIcon from '@mui/icons-material/CarRepairOutlined';
import PrecisionManufacturingOutlinedIcon from '@mui/icons-material/PrecisionManufacturingOutlined';
import WarningIcon from '@mui/icons-material/Warning';
import CheckBoxIcon from '@mui/icons-material/CheckBox';
import RefreshIcon from '@mui/icons-material/Refresh';
import PowerIcon from '@mui/icons-material/Power';
import PowerOffIcon from '@mui/icons-material/PowerOff';

import Grid from '@mui/material/Grid';
import Paper from '@mui/material/Paper';

import { experimentalStyled as styled } from '@mui/material/styles';
import RadioGroup from '@mui/material/RadioGroup';
import FormControlLabel from '@mui/material/FormControlLabel';
import Radio from '@mui/material/Radio';
import TextField from '@mui/material/TextField';
import Stack from '@mui/material/Stack';
import { VideoLabel, Link, VolumeUp, Highlight, Speaker, ToggleOn, Translate, ShowChart, TripOrigin, Timer, Handshake, KeyboardDoubleArrowUp, MusicNote, MusicOff } from '@mui/icons-material';
import { SvgIcon, Switch } from '@mui/material';
import { Audio, Puff } from 'react-loader-spinner';

const matchTimeFormatter = new Intl.NumberFormat('en-US', { maximumFractionDigits: 0 });

const darkTheme = createTheme({
  palette: {
    mode: 'dark',
  },
});

const Item = styled(Paper)(({ theme }) => ({
  backgroundColor: theme.palette.mode === 'dark' ? '#1A2027' : '#fff',
  ...theme.typography.body2,
  padding: theme.spacing(2),
  textAlign: 'center',
  color: theme.palette.text.secondary,
}));

const COLOR_GREEN = '#4caf50';
const COLOR_RED = '#b71c1c';

const OTHER_HIGLIGHT = '#35afea';

interface Topics {
  tabPub?: NetworkTablesTopic<string>;
  selectedAutoPub?: NetworkTablesTopic<string>;
  selectedScoreTypePub?: NetworkTablesTopic<string>;
  armUpPub?: NetworkTablesTopic<boolean>;
  selectedIntakePub?: NetworkTablesTopic<string>;
  selectedClimbPositionPub?: NetworkTablesTopic<string>;
  selectedSourceLanePub?: NetworkTablesTopic<string>;
  signalSelectionPub?: NetworkTablesTopic<string>;
  startShooterPub?: NetworkTablesTopic<boolean>;
}

const topics: Topics = {
  tabPub: undefined,
  selectedAutoPub: undefined,
  selectedScoreTypePub: undefined,
  armUpPub: undefined,
  selectedIntakePub: undefined,
  selectedClimbPositionPub: undefined,
  selectedSourceLanePub: undefined,
  signalSelectionPub: undefined,
  startShooterPub: undefined
};

// const NT_CORE = NetworkTables.getInstanceByTeam(1188);
const NT_CORE = NetworkTables.getInstanceByURI("localhost");

class App extends React.Component<{}, {
  connected: boolean,
  selectedTab: string,
  driverStation: number,
  joystick0: boolean,
  joystick2: boolean,
  joystick3: boolean,
  matchTime: number,
  alliance: string,
  selectedAuto: string,
  selectedAutoFromRobot: string,
  autoOptions: Array<string>,
  selectedScoreType: string,
  selectedIntakeType: string,
  selectedClimbPosition: string,
  armUp: boolean,
  armUpToggle: boolean,
  selectedSourceLane: string,
  signalSelection: string,
  startShooter: boolean
}> {
  constructor(props: {}) {
    super(props);
    this.state = {
      connected: false,
      selectedTab: "Autonomous",
      driverStation: 0,
      joystick0: false,
      joystick2: false,
      joystick3: false,
      matchTime: -1,
      alliance: "NONE",
      selectedAuto: "NONE",
      selectedAutoFromRobot: "NONE",
      autoOptions: [],
      selectedScoreType: "SPEAKER",
      selectedIntakeType: "GROUND",
      selectedClimbPosition: "LEFT_CLOSE",
      armUp: false,
      armUpToggle: false,
      selectedSourceLane: "CENTER",
      signalSelection: "NONE",
      startShooter: false
    };
  }

  setConnected(connected: boolean) {
    this.setState({
      connected: connected
    });
  }

  handleTabClick(tab: string) {
    console.log("handleTabClick: " + tab);
    this.setState({
      selectedTab: tab
    });

    topics.tabPub?.setValue(tab);
  }

  initTab(tab: string | null) {
    console.log("initTab: " + tab);
    this.setState({
      selectedTab: tab ?? "Autonomous"
    });
  }

  robotSendTab(tab: string | null) {
    console.log("robotSendTab: " + tab);
    this.setState({
      selectedTab: tab ?? "Autonomous"
    });
  }

  handleDriveStationChangeFromRobot(location: number | null) {
    this.setState({
      driverStation: location ?? 0
    });
  }

  handleStartShooterToggle(startShooter: boolean) {
    topics.startShooterPub?.setValue(startShooter);
    this.setState({
      startShooter: startShooter
    })
  }

  handleJoystickStatusUpdateFromRobot(connected: boolean | null, index: number) {
    console.log("handleJoystickStatusUpdateFromRobot: " + connected + "-" + index);
    connected = connected ?? false;
    switch(index) {
      case 0:
        this.setState({joystick0: connected});
        break;
      case 2:
        this.setState({joystick2: connected});
        break;
      case 3:
        this.setState({joystick3: connected});
        break;
      default:
        break;
    }
  }

  setMatchTime(matchTime: number | null) {
    this.setState({
      matchTime: matchTime ?? -1
    });
  }

  setAlliance(alliance: string | null) {
    alliance = alliance ?? "NONE";
    this.setState({
      alliance: alliance
    })
  }

  onOptionsChange(options: string[] | null) {
    this.setState({
      autoOptions: options ?? [""]
    })
  }

  handleRadioChange(event: React.ChangeEvent<HTMLInputElement>) {
    let value = (event.target as HTMLInputElement).value;
    console.log("handleRadioChange: " + value);
    this.setState({
      selectedAuto: value
    });

    topics.selectedAutoPub?.setValue(value);
  };

  handleArmUpSelection(enabled: boolean) {
    enabled ? this.enableArmUpAnimation() : this.disableArmUpAnimation();
    this.setState({
      armUp: enabled
    });
    topics.armUpPub?.setValue(enabled);
  }

  setSelectedAutoFromRobot(selected: string | null) {
    console.log("setSelectedAutoFromRobot: " + selected);
    let auto = selected ?? "None Selected...";
    this.setState({
      selectedAutoFromRobot: auto
    });

    this.setState({
      selectedAuto: auto
    });
  }

  disableArmUpAnimation() {
    const animatedElements : HTMLCollectionOf<SVGAnimateElement> = window.document.getElementsByTagName("animate");
    for (const index in animatedElements) {
      animatedElements.item(Number(index))?.endElement();
    }
  }

  enableArmUpAnimation() {
    const animatedElements : HTMLCollectionOf<SVGAnimateElement> = window.document.getElementsByTagName("animate");
    for (const index in animatedElements) {
      animatedElements.item(Number(index))?.beginElement();
    }
  }

  resetDashboard() {
    NT_CORE.client.cleanup();
    window.location.reload();
  }

  componentDidMount() {
    NT_CORE.addRobotConnectionListener(this.setConnected.bind(this), true);

    topics.tabPub = NT_CORE.createTopic<string>('/ReactDash/Main/dpub/tab', NetworkTablesTypeInfos.kString);
    topics.tabPub.publish({retained: true});
    topics.tabPub.subscribe((value: string | null) => { this.initTab(value); }, true);

    NT_CORE.createTopic<string>('/ReactDash/Main/rpub/goTotab', NetworkTablesTypeInfos.kString)
    .subscribe((value: string | null) => { this.robotSendTab(value); }, true);
    NT_CORE.createTopic<number>('/ReactDash/Main/rpub/driverStation', NetworkTablesTypeInfos.kInteger)
    .subscribe((value: number | null) => { this.handleDriveStationChangeFromRobot(value); }, true);
    NT_CORE.createTopic<boolean>('/ReactDash/Main/rpub/joystick0', NetworkTablesTypeInfos.kBoolean)
    .subscribe((value: boolean | null) => { this.handleJoystickStatusUpdateFromRobot(value, 0); }, true);
    NT_CORE.createTopic<boolean>('/ReactDash/Main/rpub/joystick2', NetworkTablesTypeInfos.kBoolean)
    .subscribe((value: boolean | null) => { this.handleJoystickStatusUpdateFromRobot(value, 2); }, true);
    NT_CORE.createTopic<boolean>('/ReactDash/Main/rpub/joystick3', NetworkTablesTypeInfos.kBoolean)
    .subscribe((value: boolean | null) => { this.handleJoystickStatusUpdateFromRobot(value, 3); }, true);
    NT_CORE.createTopic<number>('/ReactDash/Autonomous/rpub/matchTime', NetworkTablesTypeInfos.kDouble)
    .subscribe((value: number | null) => { this.setMatchTime(value); }, true);
    NT_CORE.createTopic<string>('/ReactDash/Autonomous/rpub/alliance', NetworkTablesTypeInfos.kString)
    .subscribe((value: string | null) => { this.setAlliance(value); }, true);

    const optionsTopic = NT_CORE.createTopic<string[]>('/ReactDash/Autonomous/rpub/options', NetworkTablesTypeInfos.kString);
    optionsTopic.subscribe((value: string[] | null) => { this.onOptionsChange(value); }, true);

    topics.selectedAutoPub = NT_CORE.createTopic<string>('/ReactDash/Autonomous/dpub/selectedAuto', NetworkTablesTypeInfos.kString, "No Auto");
    topics.selectedAutoPub.publish({retained: true}); // make us the publisher for this topic and tell the server retain the value if we disconnect
    topics.selectedScoreTypePub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/selectedScoreType', NetworkTablesTypeInfos.kString, "SPEAKER");
    topics.selectedScoreTypePub.publish({retained: true});
    topics.armUpPub = NT_CORE.createTopic<boolean>('/ReactDash/Teleop/dpub/armUp', NetworkTablesTypeInfos.kBoolean, false);
    topics.armUpPub.publish({retained: true});
    topics.selectedIntakePub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/selectedIntakeType', NetworkTablesTypeInfos.kString, "None");
    topics.selectedIntakePub.publish({retained: true});
    topics.selectedClimbPositionPub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/selectedClimbPosition', NetworkTablesTypeInfos.kString, "None");
    topics.selectedClimbPositionPub.publish({retained: true});
    topics.selectedSourceLanePub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/selectedSourceLane', NetworkTablesTypeInfos.kString, "None");
    topics.selectedSourceLanePub.publish({retained: true});
    topics.signalSelectionPub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/signalSelection', NetworkTablesTypeInfos.kString, "NONE");
    topics.signalSelectionPub.publish({retained: true});
    topics.startShooterPub = NT_CORE.createTopic<boolean>('/ReactDash/Teleop/dpub/startShooter', NetworkTablesTypeInfos.kBoolean, false);
    topics.startShooterPub.publish({retained: true});

    NT_CORE.createTopic<string>('/ReactDash/Teleop/rpub/selectedScoreType', NetworkTablesTypeInfos.kString)
    .subscribe((value: string | null) => 
      {
        console.log("rpub: " + value ?? this.state.selectedScoreType);
        this.handleScoreSelection(value ?? this.state.selectedScoreType);
      }, true);
    NT_CORE.createTopic<boolean>('/ReactDash/Teleop/rpub/armUp', NetworkTablesTypeInfos.kString)
    .subscribe((value: boolean | null) => this.handleArmUpSelection(value ?? this.state.armUp), true);
    NT_CORE.createTopic<string>('/ReactDash/Teleop/rpub/selectedIntakeType', NetworkTablesTypeInfos.kString)
    .subscribe((value: string | null) => this.handleIntakeSelection(value ?? this.state.selectedIntakeType), true);
    NT_CORE.createTopic<string>('/ReactDash/Teleop/rpub/selectedClimbPosition', NetworkTablesTypeInfos.kString)
    .subscribe((value: string | null) => this.handleClimbSelection(value ?? this.state.selectedClimbPosition), true);
    NT_CORE.createTopic<string>('/ReactDash/Teleop/rpub/selectedSourceLane', NetworkTablesTypeInfos.kString)
    .subscribe((value: string | null) => this.handleSourceSelection(value ?? this.state.selectedSourceLane), true);
    NT_CORE.createTopic<string>('/ReactDash/Teleop/rpub/signalSelection', NetworkTablesTypeInfos.kString)
    .subscribe((value: string | null) => this.handleSignalSelection(value ?? this.state.signalSelection), true);
    NT_CORE.createTopic<boolean>('/ReactDash/Teleop/rpub/startShooter', NetworkTablesTypeInfos.kString)
    .subscribe((value: boolean | null) => this.handleStartShooterToggle(value ?? this.state.startShooter), true);
    
    NT_CORE.createTopic<string>('/ReactDash/Autonomous/rpub/selectedAuto', NetworkTablesTypeInfos.kString, "No Auto")
      .subscribe((value: string | null) => { this.setSelectedAutoFromRobot(value); }, true);

  }

  componentWillUnmount() {
    NT_CORE.client.cleanup();
    console.log("componentWillUnmount");
  }

  handleScoreSelection(type: string) {
    // If the current climbing option is NOT a middle location, and we select trap as the scoring option, the climbing option should change to the middle climbing location on the selected chain
    if (type == "TRAP") {
      const selectedClimbPosition = this.state.selectedClimbPosition
      if (selectedClimbPosition == "OPPOSITE_RIGHT" || selectedClimbPosition == "OPPOSITE_LEFT") {
        this.handleClimbSelection("OPPOSITE_CENTER");
      }
      else if (selectedClimbPosition == "RIGHT_CLOSE" || selectedClimbPosition == "RIGHT_FAR") {
        this.handleClimbSelection("RIGHT_CENTER");
      }
      else if (selectedClimbPosition == "LEFT_CLOSE" || selectedClimbPosition == "LEFT_FAR") {
        this.handleClimbSelection("LEFT_CENTER");
      }
    }
    const currentSelectedButton = window.document.getElementById(type)
    console.log("selected: " + currentSelectedButton?.id)
    if (currentSelectedButton != null) currentSelectedButton.style.backgroundColor = "#43a5ff"
    const previouslySelectedButton = window.document.getElementById(this.state.selectedScoreType)
    console.log("previous: " + previouslySelectedButton?.id)
    if (previouslySelectedButton != null && previouslySelectedButton.id != type) {
      previouslySelectedButton.style.backgroundColor = "#262b32";
    }
    console.log(type)
    topics.selectedScoreTypePub?.setValue(type);
    this.setState({
      selectedScoreType: type
    });
  }

  handleSourceSelection(type: string) {
    const currentSelectedButton = window.document.getElementById(type);
    if (currentSelectedButton != null) currentSelectedButton.style.backgroundColor = "#43a5ff";
    const previouslySelectedButton = window.document.getElementById(this.state.selectedSourceLane)
    if (previouslySelectedButton != null && previouslySelectedButton.id != type) {
      window.document.getElementById(this.state.selectedSourceLane)!.style.backgroundColor = "#262b32";
    }
    topics.selectedSourceLanePub?.setValue(type);
    this.setState({
      selectedSourceLane: type
    });
  }

  handleIntakeSelection(type: string) {
    const currentSelectedButton = window.document.getElementById(type);
    if (currentSelectedButton != null) currentSelectedButton.style.backgroundColor = "#43a5ff";
    const previouslySelectedButton = window.document.getElementById(this.state.selectedIntakeType)
    if (previouslySelectedButton != null && previouslySelectedButton.id != type) {
      window.document.getElementById(this.state.selectedIntakeType)!.style.backgroundColor = "#262b32";
    }
    topics.selectedIntakePub?.setValue(type);
    this.setState({
      selectedIntakeType: type
    });
  }

  generateClimbSelections() {
    const elements = [];
    const positions = ["LEFT_FAR", "LEFT_CENTER", "LEFT_CLOSE", "RIGHT_CLOSE", "RIGHT_CENTER", "RIGHT_FAR", "OPPOSITE_RIGHT", "OPPOSITE_CENTER", "OPPOSITE_LEFT"]
    const xOffsets = [112, 148, 190, 283, 326, 362, 321, 238, 155]
    const yOffsets = [113, 177, 250, 250, 177, 113, 30, 30, 30]
    for (let i = 0; i < 9; i++) {
      elements.push(<Radio 
        sx={{
          '& .MuiSvgIcon-root': {
            fontSize: this.state.selectedClimbPosition == positions[i] ? 60 : null,
            transform: this.state.selectedClimbPosition == positions[i] ? 'translate(-15px, -15px)' : null
          },
        }}
        id={positions[i]} 
        checked={this.state.selectedClimbPosition == positions[i]} 
        onChange={
          (e) => this.handleClimbSelection(e.currentTarget.id)
        } 
        style={{position: "absolute", left: xOffsets[i], top: yOffsets[i]}}/>);
    }
    return elements.map((element) => element)
  }

  handleClimbSelection(selection: string) {
    // If a non-trap (non-middle) climbing option is selected, and trap is the scoring target, change the scoring target to climb
    if (selection != "LEFT_CENTER" && selection != "RIGHT_CENTER" && selection != "OPPOSITE_CENTER" && this.state.selectedScoreType == "TRAP") {
      this.handleScoreSelection("CLIMB");
    }
    topics.selectedClimbPositionPub?.setValue(selection);
    this.setState({selectedClimbPosition: selection})
  }

  handleSignalSelection(type: string) {
    const deselectedButton : boolean = type == this.state.signalSelection
    const currentSelectedButton = window.document.getElementById(type);
    if (deselectedButton) {
      if (currentSelectedButton != null) currentSelectedButton.style.backgroundColor = "#262b32";
      topics.signalSelectionPub?.setValue("NONE");
      this.setState({
        signalSelection: "NONE"
      });
      return;
    }
    if (currentSelectedButton != null) currentSelectedButton.style.backgroundColor = "#43a5ff";
    const previouslySelectedButton = window.document.getElementById(this.state.signalSelection)
    if (previouslySelectedButton != null) {
      previouslySelectedButton.style.backgroundColor = "#262b32";
    }
    topics.signalSelectionPub?.setValue(type);
    this.setState({
      signalSelection: type
    });
  }
  
  render() {
    let allianceColor = '#808080'
    switch(this.state.alliance) {
      case "Blue":
        allianceColor = '#1565c0'
        break;
      case "Red":
        allianceColor = '#ff1744'
        break;
      default:
    }

    return (
      <ThemeProvider theme={darkTheme}>
        <CssBaseline />
        <Grid container component="main" sx={{ height: '100vh', paddingRight: darkTheme.spacing(2), paddingTop: darkTheme.spacing(2) }} columns={{ xs: 12, sm: 12, md: 12 }} spacing={2}>
          <Grid item  xs={2} sm={2} md={2}>
            <ListItemButton selected={this.state.selectedTab == "Autonomous"} onClick={() => this.handleTabClick("Autonomous")}>
              <ListItemIcon>
                <SmartToyOutlinedIcon />
              </ListItemIcon>
              <ListItemText primary="Autonomous" />
            </ListItemButton>
            <Divider sx={{ my: 1 }} />
            <ListItemButton selected={this.state.selectedTab == "Teleop"} onClick={() => this.handleTabClick("Teleop")}>
              <ListItemIcon>
                <SportsEsportsOutlinedIcon />
              </ListItemIcon>
              <ListItemText primary="Teleop" />
            </ListItemButton>
            <Divider sx={{ my: 1 }} />
            <ListItemButton selected={this.state.selectedTab == "Overrides"} onClick={() => this.handleTabClick("Overrides")}>
              <ListItemIcon>
                <ToggleOn />
              </ListItemIcon>
              <ListItemText primary="Overrides" />
            </ListItemButton>
            <Divider sx={{ my: 1 }} />
            <ListItemButton onClick={() => this.resetDashboard()}>
              <ListItemIcon>
                <RefreshIcon />
              </ListItemIcon>
              <ListItemText primary="Refresh" />
            </ListItemButton>
          </Grid>
          <Grid item xs={10} sm={10} md={10}>
            <Grid container columns={{ xs: 9, sm: 9, md: 9 }} spacing={2}>
              <Grid item xs={9} sm={9} md={9}>
                <Stack direction="row" alignItems="stretch" justifyContent="space-evenly" spacing={2}>
                  <Item sx={{width: '100%', fontSize: '2rem', backgroundColor: allianceColor}}>
                    {this.state.alliance} Alliance
                  </Item>
                  <Item sx={{width: '100%', fontSize: '2rem'}}>
                      Driver Station: {this.state.driverStation}
                  </Item>
                  <Item sx={{padding: '0px', width: '100%'}}>
                    <SportsEsportsOutlinedIcon sx={{fontSize: '5rem', color: this.state.joystick0 ? COLOR_GREEN : COLOR_RED}} />
                    <SportsEsportsOutlinedIcon sx={{fontSize: '5rem', color: this.state.joystick2 ? COLOR_GREEN : COLOR_RED}} />
                    <SportsEsportsOutlinedIcon sx={{fontSize: '5rem', color: this.state.joystick3 ? COLOR_GREEN : COLOR_RED}} />
                  </Item>
                  <Item>{this.state.connected ? <PowerIcon sx={{fontSize: '3rem', color: COLOR_GREEN}} /> : <PowerOffIcon sx={{fontSize: '3rem', color: COLOR_RED}} />}</Item>
                  <Item sx={{fontSize: '3rem', width: '100%', padding: darkTheme.spacing(1)}}>
                    {this.state.connected ? matchTimeFormatter.format(this.state.matchTime) : (<span style={{color: COLOR_RED}}>disconnected!</span>)}
                  </Item>
                </Stack>
              </Grid>
              {this.state.selectedTab == "Autonomous" &&
                <React.Fragment>
                  <Grid item xs={5} sm={5} md={5}>
                    <RadioGroup
                      value={this.state.selectedAuto}
                      onChange={this.handleRadioChange.bind(this)}
                    >
                      {this.state.autoOptions.map((option, _) => (
                        <FormControlLabel value={option} control={<Radio />} label={option} componentsProps={{typography: {fontSize: '1.3rem'}}} />
                      ))}
                    </RadioGroup>
                  </Grid>
                  <Grid item xs={4} sm={4} md={4}>
                    <TextField
                      fullWidth
                      disabled
                      label="Dashboard Selected Autonomous"
                      value={this.state.selectedAuto}
                      sx={{marginBottom: darkTheme.spacing(2)}}
                      InputProps={{sx:{fontSize: '2rem'}}}
                    />
                    {this.state.selectedAuto === this.state.selectedAutoFromRobot
                      ? <CheckBoxIcon sx={{fontSize: 50, color: COLOR_GREEN}} />
                      : <WarningIcon sx={{fontSize: 50, color: COLOR_RED}} />
                    }
                    <TextField
                      fullWidth
                      disabled
                      label="Robot Selected Autonomous"
                      value={this.state.selectedAutoFromRobot}
                      sx={{marginTop: darkTheme.spacing(2)}}
                      InputProps={{sx:{fontSize: '2rem'}}}
                    />
                  </Grid>
                </React.Fragment>
              }
              {this.state.selectedTab == "Teleop" &&
                <React.Fragment>
                  <Grid container spacing={2} columns={3}>
                    <Grid item>
                      <Stack marginTop={2} marginLeft={2}>
                        <p style={{marginBottom: 0}}>Scoring Selection</p>    
                      </Stack>              
                      <Stack height={'100'} direction={'row'} spacing={2} marginTop={2} marginLeft={2} width={"100%"}>
                        <Stack>
                          <Item id="SPEAKER" style={{backgroundColor: "#43a5ff"}} onClick={(e) => this.handleScoreSelection("SPEAKER")}><p>Speaker</p><img width="90" src="./speaker.png"/></Item>
                        </Stack>
                        <Stack>
                          <Item id="AMP" onClick={(e) => this.handleScoreSelection("AMP")}><p>Amp</p><KeyboardDoubleArrowUp style={{fontSize: 80}}/></Item>
                        </Stack>
                        <Stack >
                          <Item id="TRAP" onClick={(e) => this.handleScoreSelection("TRAP")}><p>Trap</p><VideoLabel style={{fontSize: 80}}/></Item>
                        </Stack>
                        <Stack>
                          <Item id="CLIMB" onClick={(e) => this.handleScoreSelection("CLIMB")}><p>Climb</p><Link style={{fontSize: 80}}/></Item>
                        </Stack>
                      </Stack>
                      <Stack position="relative">
                        <Stack marginTop={4} alignItems={"center"}>
                          {this.generateClimbSelections()}
                          <img width={350} src={this.state.alliance == "Red" ? "./stage_red.png" : "./stage_blue.png"}></img>
                        </Stack>
                      </Stack>
                    </Grid>
                    <Grid item width={400}>
                      <Stack marginTop={2} marginLeft={2}>
                        <p style={{marginBottom: 0}}>Intake Selection</p>    
                      </Stack>              
                      <Stack height={'100'} direction={'row'} spacing={2} marginTop={2} marginLeft={2} width={"100%"}>
                        <Stack>
                          <Item id="GROUND" style={{backgroundColor: "#43a5ff"}} onClick={() => this.handleIntakeSelection("GROUND")}><p>Ground</p><TripOrigin style={{fontSize: 80}}/></Item>
                        </Stack>
                        <Stack>
                          <Item id="SOURCE" onClick={() => this.handleIntakeSelection("SOURCE")}><p>Source</p><ShowChart style={{fontSize: 80}}/></Item>
                        </Stack>
                        <Stack>
                          <Item id="TRAP_SOURCE" onClick={() => this.handleIntakeSelection("TRAP_SOURCE")}><p>Trap Source</p><VideoLabel style={{fontSize: 80}}/></Item>
                        </Stack>
                      </Stack>
                      <Stack marginTop={2} marginLeft={2}>
                        <p style={{marginBottom: 0}}>Trap Source Lane Selection</p>    
                      </Stack>
                      <Stack width={'100%'} direction="column" spacing={2} marginTop={2} marginLeft={2}>
                        <Stack direction="row" spacing={2}>
                          <Item id="LEFT" onClick={() => this.handleSourceSelection("LEFT")}>Left Source</Item>
                          <Item id="CENTER" style={{backgroundColor: "#43a5ff"}} onClick={() => this.handleSourceSelection("CENTER")}>Center Source</Item>
                          <Item id="RIGHT" onClick={() => this.handleSourceSelection("RIGHT")}>Right Source</Item>
                        </Stack>
                        <Stack direction={'column'} marginTop={2} marginLeft={2} >
                          <p style={{marginBottom: 0}}>Shooter Rev</p>
                          <Stack direction={'row'} marginTop={2}>
                            <Item style={{backgroundColor: this.state.startShooter ? "#43a5ff" : "#262b32"}} id="shooter-rev" onClick={(e) => this.handleStartShooterToggle(!this.state.startShooter)}><p>Shooter Rev</p>{this.state.startShooter ? <MusicNote style={{fontSize: 80}}/> : <MusicOff style={{fontSize: 80}}/>}</Item>
                          </Stack>
                        </Stack>
                      </Stack>
                    </Grid>
                    <Grid item>
                      <Stack>
                        <Stack marginTop={2} marginLeft={2}>
                          <p style={{marginBottom: 0}}>Robot Signaling</p>    
                        </Stack>             
                        <Stack height={'100'} direction={'row'} spacing={2} marginTop={2} marginLeft={2} width={"100%"}>
                          <Item id="CO_OP_SIGNAL" onClick={(e) => this.handleSignalSelection("CO_OP_SIGNAL")}>
                            <p>Co-Op</p>                        
                            <img width="80" height="80" src="./handshake.svg"/>
                          </Item>
                          <Item id="AMP_SIGNAL" onClick={(e) => this.handleSignalSelection("AMP_SIGNAL")}>
                            <p>Amp</p>
                            <Timer style={{fontSize: 80}}/>
                          </Item>
                        </Stack>
                      </Stack>
                      <Stack position={"relative"} marginLeft={2} marginTop={2} spacing={2}>
                        <Stack>
                          <p style={{marginBottom: 0}}>Arm Up</p>  
                        </Stack>
                        <Stack style={{transform: 'translate(-50%, -50%)'}} position="absolute" top="50%" left="50%" height={'100'} direction={'column'} alignItems={'center'} marginTop={2} spacing={4}>
                            <Switch onChange={(e) => {
                              this.handleArmUpSelection(e.target.checked);
                              this.setState({
                                armUpToggle: e.target.checked
                              });
                            }}/>
                            <Item 
                              style={{backgroundColor: this.state.armUp ? "#43a5ff" : "#262b32"}} 
                              // Touch screen
                              onTouchStart={() => !this.state.armUpToggle ? this.handleArmUpSelection(true): null} 
                              onTouchEnd={() => !this.state.armUpToggle ? this.handleArmUpSelection(false) : null} 
                              // Laptop
                              onMouseDown={() => !this.state.armUpToggle ? this.handleArmUpSelection(true): null} 
                              onMouseUp={() => !this.state.armUpToggle ? this.handleArmUpSelection(false) : null} 
                            >
                              <p style={{margin: 0}}>Temporary Enable</p>
                            </Item>
                        </Stack>
                        {this.state.armUp ? <Puff
                          height="300"
                          width="300"
                          color={COLOR_RED}
                          ariaLabel="audio-loading"
                          wrapperStyle={{}}
                          wrapperClass="wrapper-class"
                          visible={true}
                          /> : <div style={{border: "12px solid #bec0c2", height: 300, width: 300, borderRadius: 150}}></div>}
                      </Stack>
                    </Grid>
                  </Grid>
                </React.Fragment>
              }
              {this.state.selectedTab == "Overrides" &&
                <React.Fragment>

                </React.Fragment>
              }
            </Grid>
          </Grid>
        </Grid>
      </ThemeProvider>
    );
  }
}

export default App;