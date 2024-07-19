import { Injectable } from '@angular/core';
import { HttpClient, HttpErrorResponse } from "@angular/common/http";
import {catchError, throwError} from "rxjs";
import { Coordinates } from 'src/app/interfaces/Coordinates';
import { Quaternion } from 'src/app/interfaces/Quaternion';

const BASE_URL = 'http://localhost:3000/api';

@Injectable({
  providedIn: 'root'
})
export class CommunicationService {
  constructor(private readonly http: HttpClient) { }

  identify(robot_id = 'all') {
    return this.http.post<any>( BASE_URL + '/command/identify', { robot_id }).pipe(
      catchError(this.handleError)
    );
  }

  startMission() {
    return this.http.post<any>( BASE_URL + '/command/start_mission', {}).pipe(
      catchError(this.handleError)
    );
  }

  stopMission() {
    return this.http.post<any>( BASE_URL + '/command/stop_mission', {}).pipe(
      catchError(this.handleError)
    );
  }

  setInitPositions(initPositions: Map<string, { init_position: Coordinates; init_orientation: Quaternion }>) {
    const converted = Object.fromEntries(initPositions);
    return this.http.post<any>( BASE_URL + '/info/init_positions', converted).pipe(
      catchError(this.handleError)
    );
  }

  private handleError(error: HttpErrorResponse) {
    if (error.error instanceof ErrorEvent) {
      console.error('An error occurred:', error.error.message);
    } else {
      console.error(error.error)
    }
    return throwError('Something bad happened; please try again later.');
  }
}
