import { subscribeToLogEvents } from './EventEmitter';
import { SocketManagerService } from '../services/socket-manager.service';

export default class LogSocketManager {
    constructor(private readonly socketManagerService: SocketManagerService) {
        subscribeToLogEvents(this.handleLogEvent.bind(this));
    }

    private handleLogEvent(logData: any) {
        this.socketManagerService.sio.emit('log', logData);
    }
}
