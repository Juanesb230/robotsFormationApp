import {combineReducers} from 'redux'
import ipReducer from './ipReducer'
import portReducer from './portReducer'
import vReducer from './vReducer'
import wReducer from './wReducers'
import top_velReducer from './topic_velReducer'
import rosReducer from './rosReducer'
import connectionReducer from './connectionReducer'

const rootReducer = combineReducers({
    ipID: ipReducer,
    portID: portReducer,
    vrefID: vReducer,
    wrefID: wReducer,
    top_velID: top_velReducer,
    rosID: rosReducer,
    rosconID: connectionReducer
})

export default rootReducer