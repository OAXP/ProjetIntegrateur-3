import { NgModule } from '@angular/core';
import { BrowserModule } from '@angular/platform-browser';
import { HttpClientModule } from '@angular/common/http';
import { AppComponent } from './app.component';
import { BrowserAnimationsModule } from '@angular/platform-browser/animations';
import { MatIconModule } from '@angular/material/icon';
import { MatToolbarModule } from '@angular/material/toolbar';
import { MatButtonModule } from '@angular/material/button';
import { MatCardModule } from '@angular/material/card';
import { DragDropModule } from '@angular/cdk/drag-drop';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { SystemInfoComponent } from './components/system-info/system-info.component';
import { SystemRuntimeComponent } from './components/system-runtime/system-runtime.component';
import { XtermComponent } from './components/xterm/xterm.component';
import { AppRoutingModule } from './app-routing.module';
import { StatusPageComponent } from './pages/status-page/status-page.component';
import { ConfigPageComponent } from './pages/config-page/config-page.component';
import { MapPageComponent } from './pages/map-page/map-page.component';
import { LocateComponent } from './components/locate/locate.component';
import { GridsterModule } from 'angular-gridster2';
import { MatListModule } from '@angular/material/list';
import { MissionHistoryComponent } from './components/mission-history/mission-history.component';
import { MatTableModule } from '@angular/material/table';
import { HistoryPageComponent } from './pages/history-page/history-page.component';
import { MatMenuModule } from '@angular/material/menu';
import { MissionControlComponent } from './components/mission-control/mission-control.component';
import { LoginPageComponent } from './pages/login-page/login-page.component';
import { FormsModule } from '@angular/forms';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatDialogModule } from '@angular/material/dialog';
import { MapComponent } from './components/map/map.component';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { InitPositionComponent } from './components/init-position/init-position.component';
import {MatRadioButton, MatRadioGroup} from "@angular/material/radio";
import { MatSidenavModule } from '@angular/material/sidenav';

@NgModule({
  declarations: [
    AppComponent,
    SystemInfoComponent,
    SystemRuntimeComponent,
    XtermComponent,
    StatusPageComponent,
    ConfigPageComponent,
    MapPageComponent,
    LocateComponent,
    MissionHistoryComponent,
    HistoryPageComponent,
    MissionControlComponent,
    LoginPageComponent,
    MapComponent,
    InitPositionComponent
  ],
  imports: [
    MatMenuModule,
    MatTableModule,
    MatListModule,
    GridsterModule,
    MatButtonToggleModule,
    DragDropModule,
    MatCardModule,
    MatButtonModule,
    MatToolbarModule,
    MatIconModule,
    BrowserModule,
    BrowserAnimationsModule,
    AppRoutingModule,
    HttpClientModule,
    FormsModule,
    MatFormFieldModule,
    MatInputModule,
    MatProgressSpinnerModule,
    MatDialogModule,
    MatRadioButton,
    MatRadioGroup,
    MatSidenavModule
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule {
  constructor() { }
}
