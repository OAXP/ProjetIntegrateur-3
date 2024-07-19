import { NgModule } from '@angular/core';
import { CommonModule } from '@angular/common';
import { RouterModule, Routes } from '@angular/router';
import { StatusPageComponent } from './pages/status-page/status-page.component';
import { ConfigPageComponent } from './pages/config-page/config-page.component';
import { MapPageComponent } from './pages/map-page/map-page.component';
import { HistoryPageComponent } from './pages/history-page/history-page.component';
import { LoginPageComponent } from './pages/login-page/login-page.component';
import { authGuard } from './auth-guard.service';

const routes: Routes = [
  { path: 'status', component: StatusPageComponent },
  { path: 'config', component: ConfigPageComponent },
  { path: 'map', component: MapPageComponent },
  { path: 'history', component: HistoryPageComponent },
  { path: 'login', component: LoginPageComponent },
  { path: '', redirectTo: '/login', pathMatch: 'full' } // Redirection par défaut
];

@NgModule({
  declarations: [],
  imports: [
    CommonModule,
    RouterModule.forRoot(routes),
  ],
  exports: [RouterModule]
})

export class AppRoutingModule { }
