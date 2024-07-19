import { Injectable } from '@angular/core';
import { RobotInfo } from 'src/app/interfaces/RobotInfo';
import { BehaviorSubject } from 'rxjs';

@Injectable({
  providedIn: 'root'
})
export class InfoService {
  private robots: Map<string, RobotInfo>;
  private robotsSubject: BehaviorSubject<Map<string, RobotInfo>>;
  private mapSubject: BehaviorSubject<string>;

  constructor() {
    this.robots = new Map();
    this.robotsSubject = new BehaviorSubject(new Map());
    this.mapSubject = new BehaviorSubject('');
  }

  get robotInfos(): Map<string, RobotInfo> {
    return this.robots;
  }

  updateRobotInfo(robotId: string, robotInfo: RobotInfo) {
    this.robots.set(robotId, robotInfo);
    this.robotsSubject.next(this.robots);
  }

  get robotInfosObservable() {
    return this.robotsSubject.asObservable();
  }

  get mapInfo(): string {
    return this.mapSubject.value;
  }

  set mapInfo(value: string) {
    this.mapSubject.next(value);
  }

  get mapInfoObservable() {
    return this.mapSubject.asObservable();
  }
}
