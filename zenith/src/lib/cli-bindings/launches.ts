import { Command } from '@tauri-apps/api/shell';

export type Node = {
	package: string;
	executable: string;
};

type LaunchListRawResponse = {
	[key: string]: {
		metadata: {
			name: string;
		};
		nodes: Node[];
	};
};

export type LaunchListEntry = {
	path: string;
	metadata: {
		name: string;
	};
	nodes: Node[];
};

export type LaunchListResponse = LaunchListEntry[];

export async function launchList(launchDirectory: string): Promise<LaunchListResponse> {
	const cmd = Command.sidecar('bin/zenith-cli', ['launch', 'list', launchDirectory]);
	const child = await cmd.execute();

	if (child.code != 0) {
		throw new Error(`failed to list launches: ${child.stderr}`);
	}

	const output: LaunchListRawResponse = JSON.parse(child.stdout);
	const launches: LaunchListResponse = [];

	for (const launchFilePath in output) {
		// const launchFilePath = key;
		const launch: LaunchListEntry = {
			path: launchFilePath,
			metadata: output[launchFilePath].metadata,
			nodes: output[launchFilePath].nodes
		};
		launches.push(launch);
	}

	return launches;
}

export type RawSubsystemResponse = {
	[subsystemName: string]: {
		[packageName: string]: string[]; // List of executables within a package.
	};
};

export type PackageInfo = {
	name: string;
	executables: string[];
};

export type SubsystemInfo = {
	name: string;
	packages: PackageInfo[];
};

export type SubsystemResponse = {
	subsystems: SubsystemInfo[];
};

export async function subsystemList(): Promise<SubsystemResponse> {
	const cmd = Command.sidecar('bin/zenith-cli', ['subsystem']);
	const child = await cmd.execute();

	if (child.code != 0) {
		throw new Error(`failed to list subsystems: ${child.stderr}`);
	}

	// Parse the raw response into JSON.
	const rawResponse: RawSubsystemResponse = JSON.parse(child.stdout);

	// Convert the raw response into a structured response.
	const response: SubsystemResponse = {
		subsystems: []
	};

	for (const subsystemName in rawResponse) {
		const subsystem: SubsystemInfo = {
			name: subsystemName,
			packages: []
		};

		for (const packageName in rawResponse[subsystemName]) {
			const packageInfo: PackageInfo = {
				name: packageName,
				executables: rawResponse[subsystemName][packageName]
			};
			subsystem.packages.push(packageInfo);
		}
		response.subsystems.push(subsystem);
	}

	return response;
}
