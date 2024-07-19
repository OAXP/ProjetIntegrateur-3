import { promises as fs } from 'fs';
import { Service } from "typedi";
import * as rclnodejs from 'rclnodejs';

const FILE_PATH = './src/assets/explorations/';

@Service()
export class FileService {
    private currentExplorationId: string;

    private logFolderPath: string;

    constructor() {
        this.logFolderPath = '';
    }

    get isExploring(): boolean {
        return this.logFolderPath !== '';
    }

    async startExploration(robotInfo: Map<string, rclnodejs.communication_interfaces.msg.RobotInfo>) {
        this.currentExplorationId = `exploration-${await this.getExplorationNumber()}`;
        await fs.mkdir(FILE_PATH + `${this.currentExplorationId}`, { recursive: true });
        this.logFolderPath = FILE_PATH + `${this.currentExplorationId}/log.txt`;
        await fs.writeFile(this.logFolderPath, '');

        const robots = Array.from(robotInfo.keys());
        let type = 'Erreur';
        if (robots.length > 0) {
            type = robots[0].startsWith('limo') ? 'Simulation' : 'Physique';
        }

        const info = {
            missionId: this.currentExplorationId,
            date: new Date().toISOString().split('T')[0],
            time: new Date().toLocaleTimeString('fr-CA', { hour12: false }),
            robots,
            type: type,
        }
        await fs.writeFile(FILE_PATH + `${this.currentExplorationId}/info.json`, JSON.stringify(info));
    }

    endExploration() {
        this.logFolderPath = '';
    }

    writeLog(message: string) {
        if (this.logFolderPath) {
            const formattedMessage = message.endsWith('\n') ? message : `${message}\n`;
            fs.appendFile(this.logFolderPath, formattedMessage);
        }
    }

    async deleteExploration(id: number) {
        await fs.rmdir(FILE_PATH + `exploration-${id}`, { recursive: true });
    }

    private async getExplorationNumber(): Promise<number> {
        try {
            const dirents = await fs.readdir(FILE_PATH, { withFileTypes: true });
            const folderNumbers = dirents
                .filter(dirent => dirent.isDirectory())
                .map(dirent => {
                    const match = dirent.name.match(/exploration-(\d+)/);
                    return match ? parseInt(match[1], 10) : 0;
                });

            if (folderNumbers.length === 0) {
                return 1;
            }

            const maxNumber = Math.max(...folderNumbers);
            return maxNumber + 1;
        } catch (_) {
            return 1;
        }
    }

}
