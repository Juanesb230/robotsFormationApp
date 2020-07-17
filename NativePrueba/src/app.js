import React from 'react'
import {Image} from 'react-native'
import {createAppContainer} from 'react-navigation'
import {createDrawerNavigator, DrawerNavigatorItems, DrawerItems} from 'react-navigation-drawer'
import HomeNavigator from './modules/Login/components/HomeNavigation'
import TeleNavigator from './modules/Login/components/TeleNavigation'
import Entypo from 'react-native-vector-icons/Entypo'
import MaterialIcons from 'react-native-vector-icons/MaterialIcons'
import { Container, Body, Header} from 'native-base'

const DrawerContent = (props) => {
  return(
      <Container>
        <Header style={{ height:200}}>
          <Body >
            <Image style={{width:150, height:150, marginLeft:50}} source={require('./images/robot.png')}></Image>
          </Body>
        </Header>
        <DrawerNavigatorItems {...props}/>
      </Container>
    )
}

const LoginNavigator = createDrawerNavigator({
  HomeNavigator:{
    screen : HomeNavigator,
    navigationOptions:{
      title: 'Home',
      drawerIcon: ({item}) => (
        <Entypo name='home' size={20} ></Entypo>
      )
    }
  },
  TeleNavigator:{
    screen : TeleNavigator,
    navigationOptions:{
      title: 'Teleoperation',
      drawerIcon: ({item}) => (
        <MaterialIcons name='gamepad' size={20} ></MaterialIcons>
      )
    }
  }
},{
  drawerPosition: 'left',
  contentComponent: DrawerContent
});

export default createAppContainer(LoginNavigator)