import { Injectable } from '@angular/core';
import { io, Socket } from 'socket.io-client';
import { SERVER_BASE_URL } from '../../utils/constants';
import { InfoService } from '../info/info.service';
import { RobotInfo } from 'src/app/interfaces/RobotInfo';
import { XtermLoggerService } from '../xterm/xterm-logger.service';
import { Subject } from 'rxjs';
import { Buffer } from 'buffer';
import { UserManagerService } from '../user-manager.service';
import { User } from 'src/app/interfaces/User';

@Injectable({
  providedIn: 'root',
})
export class SocketClientService {
  private clientSocket!: Socket;
  private username!: string;
  public OnConnectionChanged = new Subject<boolean>();
  public OnLoginSuccess = new Subject<void>();
  public OnLoginError = new Subject<string>();
  public isTestingEnvironment = false;

  constructor(private infoService: InfoService, private xtermLogger: XtermLoggerService, private userService: UserManagerService) { }

  get socket(): Socket {
    return this.clientSocket;
  }

  set socket(value: Socket) {
    this.clientSocket = value;
  }

  public onError = new Subject<string>();

  async connectWithUsername(username: string) {
    if (!this.isSocketAlive()) {
      if (!this.isTestingEnvironment) {
        this.clientSocket = io(SERVER_BASE_URL, { transports: ['websocket'] });
      }

      this.clientSocket.on('connect', () => {
        this.xtermLogger.logToXterm('Attempting to register user...', 'info');
        this.clientSocket.emit('register_user', username);
      });

      this.clientSocket.on('user_registered', (data) => {
        this.OnLoginSuccess.next();
        this.xtermLogger.logToXterm(`User ${data.username} registered successfully`, 'success');
        this.username = data.username;
        localStorage.setItem('username', data.username);
        this.OnConnectionChanged.next(true);
        this.handleSockets();
      });

      this.clientSocket.on('registration_failed', (message) => {
        this.OnLoginError.next(message.error);
        this.xtermLogger.logToXterm(`Registration error: ${message.error}`, 'error');
        this.OnConnectionChanged.next(false);
      });

      this.clientSocket.on('disconnect', () => {
        this.xtermLogger.logToXterm('Disconnected', 'warning');
        this.userService.clearUser();
        this.OnConnectionChanged.next(false);
        localStorage.removeItem('username');
      });

      this.clientSocket.on('connect_error', (error) => {
        this.xtermLogger.logToXterm(`Connection error: ${error}`, 'error');
        this.OnConnectionChanged.next(false);
      });

      this.initializeEventListeners();
    }
  }

  isSocketAlive() {
    return this.clientSocket && this.clientSocket.connected;
  }

  disconnect() {
    this.clientSocket.disconnect();
  }

  handleSockets() {
    this.clientSocket.on('robot_info', (robotInfo: RobotInfo) => this.onRobotInfo(robotInfo));
    this.clientSocket.on('map', (imgBuffer: Buffer) => this.onMapInfo(imgBuffer));
    this.clientSocket.on('log', (logInfo: string) => this.onLogInfo(logInfo));
  }

  onRobotInfo(robotInfo: RobotInfo) {
    this.infoService.updateRobotInfo(robotInfo.robot_id, robotInfo);
  }

  onMapInfo(imgBuffer: Buffer) {
    this.infoService.mapInfo = 'data:image/jpeg;base64,' + btoa(
      new Uint8Array(imgBuffer).reduce((data, byte) => data + String.fromCharCode(byte), '')
    );
  }

  onLogInfo(logInfo: any) {
    switch (logInfo.level) {
      case 'info':
        console.info(logInfo.message);
        break;
      case 'error':
        console.error(logInfo.message);
        break;
      default:
        console.log(logInfo.message);
    }
  }

  private initializeEventListeners(): void {
    this.clientSocket.on('user_connected', (message: string) => {
      this.xtermLogger.logToXterm(message, 'info');
    });

    this.clientSocket.on('user_disconnected', (message: string) => {
      this.xtermLogger.logToXterm(message, 'info');
    });

    this.clientSocket.on('set_admin', (isAdmin: boolean) => {
      const user: User = {
        username: this.username,
        isAdmin: isAdmin
      };
      this.userService.setUser(user);
    });
  }
}

