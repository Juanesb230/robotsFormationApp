import React, { Component } from 'react'
import {StyleSheet, ScrollView, Alert} from 'react-native'
import { Container, Header, Left, Body, Right, Button, Title, Item,Input, Text } from 'native-base';
import AntDesign from 'react-native-vector-icons/AntDesign';
import Entypo from 'react-native-vector-icons/Entypo';
import * as actions from '../../../actions'
import {connect} from 'react-redux'
import * as ROSLIB from 'roslib'

class AnatomyExample extends Component {

  constructor(props){
    super(props)
    this.state = {
      connected: false,
      error: undefined,
    }
  }
  
  componentDidMount = () => {
    this.props.ros_connection(this.state.connected)
  }
  
  registrer = () => {
    this.props.navigation.openDrawer()  
  }

  registrer2 = () => {
    this.props.navigation.navigate('HomeSettings')
  }

  save_ip = () => {
    this.ros = new ROSLIB.Ros({url: "ws://" + this.props.ipID + ":" + this.props.portID})
    this.props.ros_node(this.ros)
    this.ros.on ('connection', () => {
      Alert.alert("Success","Connected to server: ws://" + this.props.ipID + ":" + this.props.portID)
      this.state.connected = true
      this.props.ros_connection(this.state.connected)
    })
    this.ros.on ('error', (error) => {
      Alert.alert("Error","Type error: " + error)
      this.state.connected = false
      this.props.ros_connection(this.state.connected)
    })
    this.ros.on ('close', () => {
      Alert.alert("Closed connection","The server: "+ this.props.ipID + ":" + this.props.portID + " was close")
      this.state.connected = false
      this.props.ros_connection(this.state.connected)
    })
    
  }


  render() {

    return (
        <Container>
        <Header>
          <Left>
            <Button transparent onPress={this.registrer}>
            <Entypo name='menu' size={25} style={{color : 'white'}}/>
            </Button>
          </Left>
          <Body>
            <Title>Home</Title>
          </Body>
          <Right>
            <Button transparent onPress={this.registrer2}>
              <AntDesign name='setting' size={25} style={{color : 'white'}}/>
            </Button>
          </Right>
        </Header>
          <ScrollView contentContainerStyle={styles.IP}>
            <Text style={{fontSize: 30, marginBottom:'5%',textAlign:'center', fontWeight: 'bold'}}>Remote ROS</Text>
            <Text style={{fontSize: 30, marginBottom:'15%',textAlign:'center', fontWeight: 'bold'}}>Mobile Robot Control</Text>
            <Text style={{fontSize: 20, marginBottom:'5%'}}>IP Adress:</Text>
            <Item rounded style={{width: '50%', borderColor:'black'}}>
              <Input value={this.props.ipID} placeholder='e.g. 100.0.0.0' onChangeText={(ip) => this.props.ip_master(ip)} style={{textAlign:'center'}}/>
            </Item>
            <Text style={{fontSize: 20, marginTop:'5%'}}>Port:</Text>
            <Item rounded style={{width: '50%', borderColor:'black', marginTop:'5%'}}>
              <Input value={this.props.portID} keyboardType='phone-pad' placeholder='e.g. 8081' onChangeText={(port) => this.props.port_master(port)} style={{textAlign:'center'}}/>
            </Item>
            <Button onPress={this.save_ip} rounded primary style={{width: '35%',marginLeft: '33%', marginTop:'5%'}}>
              <Text style={{marginLeft:'15%'}}>Connect</Text>
            </Button>
          </ScrollView>
      </Container>
    );
  }
}

const styles = StyleSheet.create({
  IP:{
    flexGrow: 1,
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
  }
})

const mapStateToProps = state => {
  return {
    ipID: state.ipID,
    portID: state.portID,
  }
}

export default connect(mapStateToProps,actions)(AnatomyExample)