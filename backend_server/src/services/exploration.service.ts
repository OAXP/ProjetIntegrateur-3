import { Request, Response } from 'express';
import { Service } from 'typedi';
import { promises as fs } from 'fs';
import { FileService } from "@src/services/file.service";
import Logger from "@src/utils/Logger";
import path = require('path');

const EXPLORATION_FOLDER = './src/assets/explorations';

@Service()
export class ExplorationService {
    private readonly LOGGER: Logger;

    constructor(private readonly fileService: FileService) {
        this.LOGGER = new Logger('ExplorationService', fileService);
    }

    async getAllExplorations() {
        const missions = [];
        try {
            const directories = await fs.readdir(EXPLORATION_FOLDER, { withFileTypes: true });
            for (const dirent of directories.filter(dirent => dirent.isDirectory())) {
                const missionData = await this.readMissionInfo(dirent);
                missions.push(missionData);
            }
        } catch (error) {
            console.error('Error reading missions directory:', error);
        }
        return missions;
    }

    async getLastMission() {
        let lastMissionData = null;
        try {
            const directories = await fs.readdir(EXPLORATION_FOLDER, { withFileTypes: true });
            const filteredDirectories = directories.filter(dirent => dirent.isDirectory());
            const sortedDirectories = filteredDirectories.sort((a, b) => {
                const aId = parseInt(a.name.replace('exploration-', ''), 10);
                const bId = parseInt(b.name.replace('exploration-', ''), 10);
                return bId - aId;
            });
            if (sortedDirectories.length > 0) {
                const lastDirent = sortedDirectories[0];
                lastMissionData = await this.readMissionInfo(lastDirent);
            }
        } catch (error) {
            console.error('Error reading missions directory:', error);
        }
        return lastMissionData;
    }

    async getExplorationDetails(number: number) {
        const explorationFolderPath = `${EXPLORATION_FOLDER}/exploration-${number}`;

        try {
            const [logContent, infoContent] = await Promise.all([
                this.getFileContent(`${explorationFolderPath}/log.txt`),
                this.getFileContent(`${explorationFolderPath}/info.json`),
                // TODO : fetch map image once R.F.18 is implemented
            ]);

            return {
                log: logContent,
                info: JSON.parse(infoContent),
            };
        } catch (error) {
            this.LOGGER.err(`Error retrieving exploration details for exploration-${number}:`, error);
            return null;
        }
    }

    private async readMissionInfo(dirent) {
        const infoPath = path.join(EXPLORATION_FOLDER, dirent.name, 'info.json');
        try {
            const infoContent = await fs.readFile(infoPath, 'utf-8');
            const missionInfo = JSON.parse(infoContent);
            const missionIdNumber = parseInt(dirent.name.replace('exploration-', ''), 10);
            return {
                missionId: missionIdNumber,
                date: new Date(missionInfo.date),
                time: missionInfo.time,
                type: missionInfo.type,
                robots: missionInfo.robots,
            };
        } catch (error) {
            console.error(`Error reading info.json for mission ${dirent.name}:`, error);
            return {
                missionId: parseInt(dirent.name.replace('exploration-', ''), 10),
                date: new Date(),
                time: '00:00',
                type: 'Erreur',
                robots: [],
            };
        }
    }
    

    private async getFileContent(filePath: string): Promise<string> {
        try {
            const fileContent = await fs.readFile(filePath, 'utf-8');
            return fileContent;
        } catch (error) {
            this.LOGGER.err(`Error reading file ${filePath}:`, error);
            return null;
        }
    }
}
