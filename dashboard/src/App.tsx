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
import { VideoLabel, Link, VolumeUp, Highlight, Speaker } from '@mui/icons-material';
import { Switch } from '@mui/material';

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
  selectedSourcePub?: NetworkTablesTopic<string>;
  selectedArmHeightPub?: NetworkTablesTopic<string>;
}

const topics: Topics = {
  tabPub: undefined,
  selectedAutoPub: undefined,
  selectedScoreTypePub: undefined,
  selectedSourcePub: undefined,
  selectedArmHeightPub: undefined
};

const NT_CORE = NetworkTables.getInstanceByTeam(1188);
// const NT_CORE = NetworkTables.getInstanceByURI("localhost");

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
  armHeightSelection: string,
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
      selectedScoreType: "NONE",
      armHeightSelection: "High"
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

  
  handleShooterHeightSelection(event: React.ChangeEvent<HTMLInputElement>) {
    const armHeight = event.target.checked ? "high" : "low"
    this.setState({
      armHeightSelection: armHeight
    });
    topics.selectedArmHeightPub?.setValue(armHeight);
  }

  setSelectedAuto(selected: string | null) {
    this.setState({
      selectedAuto: selected ?? "NONE"
    });
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
    topics.selectedScoreTypePub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/selectedScoreType', NetworkTablesTypeInfos.kString, "None");
    topics.selectedScoreTypePub.publish({retained: true});
    topics.selectedArmHeightPub = NT_CORE.createTopic<string>('/ReactDash/Teleop/dpub/selectedArmHeight', NetworkTablesTypeInfos.kString, "None");
    topics.selectedArmHeightPub.publish({retained: true});

    NT_CORE.createTopic<string>('/ReactDash/Autonomous/rpub/selectedAuto', NetworkTablesTypeInfos.kString, "No Auto")
      .subscribe((value: string | null) => { this.setSelectedAutoFromRobot(value); }, true);
  }

  componentWillUnmount() {
    NT_CORE.client.cleanup();
    console.log("componentWillUnmount");
  }

  handleScoreSelection(e: React.MouseEvent, type: string) {
    const elementID : string = e.currentTarget.id;
    window.document.getElementById(elementID)!.style.backgroundColor = this.state.selectedScoreType == type ? "#262b32" : "#43a5ff";
    const previouslySelectedButton = window.document.getElementById(this.state.selectedScoreType)
    if (previouslySelectedButton != null) {
      window.document.getElementById(this.state.selectedScoreType)!.style.backgroundColor = "#262b32";
    }
    const selectedScoreType = this.state.selectedScoreType == type ? "NONE" : type
    topics.selectedScoreTypePub?.setValue(selectedScoreType);
    console.log(topics.selectedScoreTypePub?.getValue())
    this.setState({
      selectedScoreType: selectedScoreType
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
                  <Stack marginTop={2} marginLeft={2}>
                    <p style={{marginBottom: 0}}>Scoring Selection</p>    
                  </Stack>              
                  <Stack height={'100'} direction={'row'} spacing={2} marginTop={2} marginLeft={2} width={"100%"}>
                    <Stack>
                      <Item id="speaker" onClick={(e) => this.handleScoreSelection(e, "speaker")}><p>Speaker</p><Speaker style={{fontSize: 100}}/></Item>
                    </Stack>
                    <Stack>
                      <Item id="amp" onClick={(e) => this.handleScoreSelection(e, "amp")}><p>Amp</p><Highlight style={{fontSize: 100}}/></Item>
                    </Stack>
                    <Stack >
                      <Item id="trap" onClick={(e) => this.handleScoreSelection(e, "trap")}><p>Trap & Climb</p><VideoLabel style={{fontSize: 100}}/></Item>
                    </Stack>
                    <Stack>
                      <Item id="climb" onClick={(e) => this.handleScoreSelection(e, "climb")}><p>Climb</p><Link style={{fontSize: 100}}/></Item>
                    </Stack>
                  </Stack>
                  <Stack direction={'column'} marginTop={2} marginLeft={2}>
                    <p style={{marginBottom: 0}}>Arm Height Selection</p>
                    <Stack height={'100'} direction={'row'} alignItems={'center'}>
                      <p>Low</p>
                      <Switch defaultChecked onChange={(e) => this.handleShooterHeightSelection(e)}/>
                      <p>High</p>
                    </Stack>
                  </Stack>
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