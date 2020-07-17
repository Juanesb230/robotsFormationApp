import {Alert} from 'react-native'
import React, { Component } from 'react';
import { Container, Header, Left, Body, Right, Button, Icon, Title, Content, Form, Item, Label, Input, Text } from 'native-base';
import * as actions from '../../../actions'
import {connect} from 'react-redux'
import * as ROSLIB from 'roslib'

class AnatomyExample extends Component {

  constructor(props){
    super(props)
    this.state = {
      topic_vel: this.props.top_velID,
      vref:'0.2',
      wref:'0.2',
      d1r: '2.0',
      d2r: '2.0',
      betar: '60',
      thetar: '0'
    }
  }

  BackTele = () => {
    this.props.navigation.navigate('Teleop')}

  save_vel = () => {
    this.props.topic_vel(this.state.topic_vel)
    this.props.vref(this.state.vref)
    this.props.wref(this.state.wref)
    Alert.alert("Success","Changed velocities")
  }

  posRef = () => {
    if (this.props.rosconID){
      this.D1ref = new ROSLIB.Param({
        ros : this.props.rosID,
        name : 'd1r'
      })
      this.D1ref.set(parseFloat(this.state.d1r))
      this.D2ref = new ROSLIB.Param({
        ros : this.props.rosID,
        name : 'd2r'
      })
      this.D2ref.set(parseFloat(this.state.d2r))
      this.Betaref = new ROSLIB.Param({
        ros : this.props.rosID,
        name : 'betar'
      })
      this.Betaref.set(parseFloat(this.state.betar)*3.1416/180)
      this.Thetaref = new ROSLIB.Param({
        ros : this.props.rosID,
        name : 'thetar'
      })
      this.Thetaref.set(parseFloat(this.state.thetar)*3.1416/180)
      Alert.alert("Success","Changed parameters")
    }
  }

  render() {
    return (
      <Container>
        <Header>
          <Left>
          <Button transparent onPress={this.BackTele}>
            <Icon name='arrow-back' />
          </Button>
          </Left>
          <Body>
            <Title style={{fontSize: 14}}>Teleoperation Settings</Title>
          </Body>
          <Right/>
        </Header>
        <Content>
          <Form>
            <Item stackedLabel >
              <Label>Velocity Topic</Label>
              <Input onChangeText={(topic_vel) => this.setState({topic_vel})}/>
            </Item>
            <Item stackedLabel >
              <Label>Lineal Velocity (m/s)</Label>
              <Input keyboardType='phone-pad' onChangeText={(vref) => this.setState({vref})}/>
            </Item>
            <Item stackedLabel last>
              <Label>Angular Velocity (rad/s)</Label>
              <Input keyboardType='phone-pad' onChangeText={(wref) => this.setState({wref})}/>
            </Item>
          </Form>
          <Button block style={{marginTop:'10%', width:'90%',marginLeft:'5%'}} onPress={this.save_vel}>
            <Text>Confirm</Text>
          </Button>
        </Content>
        <Content >
          <Form>
            <Item stackedLabel >
              <Label>Distance 1 (m)</Label>
              <Input value={this.state.xref} keyboardType='phone-pad' onChangeText={(d1r) => this.setState({d1r})}/>
            </Item>
            <Item stackedLabel last>
              <Label>Distance 2 (m)</Label>
              <Input value={this.state.yref} keyboardType='phone-pad' onChangeText={(d2r) => this.setState({d2r})}/>
            </Item>
            <Item stackedLabel >
              <Label>Angle Beta (°)</Label>
              <Input value={this.state.xref} keyboardType='phone-pad' onChangeText={(betar) => this.setState({betar})}/>
            </Item>
            <Item stackedLabel last>
              <Label>Angle Theta (°)</Label>
                <Input value={this.state.yref} keyboardType='phone-pad' onChangeText={(thetar) => this.setState({thetar})}/>
            </Item>
          </Form>
          <Button block style={{marginTop:'5%', width:'90%',marginLeft:'5%'}} onPress={this.posRef}>
            <Text>Set Position</Text>
          </Button>
          </Content>
      </Container>
    ); 
  }
}

const mapStateToProps = state => {
  return {
    top_velID: state.top_velID,
    rosID: state.rosID,
    rosconID: state.rosconID
  }
}

export default connect(mapStateToProps,actions)(AnatomyExample)